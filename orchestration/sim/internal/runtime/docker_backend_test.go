package runtime

import (
	"context"
	"errors"
	"os"
	"path/filepath"
	"reflect"
	"strings"
	"testing"
	"time"
)

type recordedCommand struct {
	command string
	args    []string
}

type fakeRunner struct {
	commands []recordedCommand
	result   CommandResult
	err      error
}

func (runner *fakeRunner) Run(ctx context.Context, command string, args ...string) (CommandResult, error) {
	_ = ctx
	runner.commands = append(runner.commands, recordedCommand{command: command, args: append([]string(nil), args...)})
	return runner.result, runner.err
}

func TestDockerServiceArgs(t *testing.T) {
	args, err := DockerServiceArgs(ServiceSpec{
		Name:          "slam",
		Image:         "navlab/slam:latest",
		ContainerName: "navlab-slam-backend",
		Command:       []string{"bash", "-lc", "echo ok"},
		Env:           map[string]string{"ROS_DOMAIN_ID": "85"},
		CWD:           "/workspace",
		Volumes:       []VolumeMount{{Source: ".", Target: "/workspace"}},
		Networks:      []string{"host"},
		Detach:        true,
	})
	if err != nil {
		t.Fatal(err)
	}
	expected := []string{
		"run",
		"--detach",
		"--name", "navlab-slam-backend",
		"--network", "host",
		"--volume", ".:/workspace",
		"--workdir", "/workspace",
		"--env", "ROS_DOMAIN_ID=85",
		"navlab/slam:latest",
		"bash", "-lc", "echo ok",
	}
	if !reflect.DeepEqual(args, expected) {
		t.Fatalf("args = %#v, want %#v", args, expected)
	}
}

func TestDockerBackendStartServiceUsesRunner(t *testing.T) {
	runner := &fakeRunner{result: CommandResult{Stdout: "container-id\n", ExitCode: 0}}
	backend := NewDockerBackend(runner)
	backend.Now = func() time.Time { return time.Date(2026, 6, 12, 1, 2, 3, 0, time.UTC) }

	handle, err := backend.StartService(ServiceSpec{
		Name:          "gazebo_sensor",
		Image:         "sensor:latest",
		ContainerName: "navlab-sensor",
		Command:       []string{"bash", "-lc", "run"},
		Detach:        true,
	})
	if err != nil {
		t.Fatal(err)
	}
	if handle.Identifier != "navlab-sensor" || handle.Backend != "docker" {
		t.Fatalf("handle = %#v", handle)
	}
	if len(runner.commands) != 2 || runner.commands[0].command != "docker" || runner.commands[1].command != "docker" {
		t.Fatalf("runner commands = %#v", runner.commands)
	}
	if !reflect.DeepEqual(runner.commands[0].args, []string{"rm", "-f", "navlab-sensor"}) {
		t.Fatalf("cleanup args = %#v", runner.commands[0].args)
	}
}

func TestDockerBackendRunProbeWritesLogAndReturnsFailure(t *testing.T) {
	logPath := filepath.Join(t.TempDir(), "probe.txt")
	runner := &fakeRunner{
		result: CommandResult{Stdout: "bad", ExitCode: 42},
		err:    errors.New("exit status 42"),
	}
	backend := NewDockerBackend(runner)

	result, err := backend.RunProbe(ProbeSpec{
		Name:    "rangefinder_probe",
		Image:   "runtime:latest",
		Command: []string{"bash", "-lc", "probe"},
		LogPath: logPath,
	})
	if err == nil {
		t.Fatal("RunProbe error = nil, want failure")
	}
	if result.ReturnCode != 42 {
		t.Fatalf("return code = %d, want 42", result.ReturnCode)
	}
	data, readErr := os.ReadFile(logPath)
	if readErr != nil {
		t.Fatal(readErr)
	}
	if string(data) != "bad" {
		t.Fatalf("log = %q, want bad", string(data))
	}
}

func TestRosbagServiceSpecReadsProfile(t *testing.T) {
	profile := filepath.Join(t.TempDir(), "topics.txt")
	if err := os.WriteFile(profile, []byte("# comment\n/tf\n/scan\n"), 0o644); err != nil {
		t.Fatal(err)
	}
	service, err := RosbagServiceSpec(RosbagSpec{
		Name:          "hover_rosbag",
		Image:         "runtime:latest",
		ContainerName: "navlab-hover-rosbag",
		TopicsProfile: profile,
		OutputPath:    "rosbag",
		DurationSec:   90,
		Storage:       "mcap",
	})
	if err != nil {
		t.Fatal(err)
	}
	if service.Name != "hover_rosbag" || service.Image != "runtime:latest" {
		t.Fatalf("service = %#v", service)
	}
	command := strings.Join(service.Command, " ")
	if !strings.Contains(command, "/tf") || !strings.Contains(command, "/scan") {
		t.Fatalf("command missing topics: %q", command)
	}
	if !strings.Contains(command, "timeout --signal=INT 90.0") {
		t.Fatalf("command missing timeout: %q", command)
	}
}

func TestSpecValidationRejectsMissingImage(t *testing.T) {
	err := ServiceSpec{Name: "bad", Command: []string{"true"}}.ValidateDocker()
	if err == nil || !strings.Contains(err.Error(), "image is required") {
		t.Fatalf("ValidateDocker() = %v, want image error", err)
	}
}
