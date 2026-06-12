package images

import (
	"bytes"
	"context"
	"errors"
	"io"
	"reflect"
	"strings"
	"testing"

	"navlab/orchestration-sim/internal/config"
)

type recordingRunner struct {
	calls []string
	err   error
}

func (runner *recordingRunner) Run(ctx context.Context, name string, args []string, stdout io.Writer, stderr io.Writer) error {
	runner.calls = append(runner.calls, name+" "+strings.Join(args, " "))
	return runner.err
}

func TestResolveBuildSpecsAllUsesConfiguredImages(t *testing.T) {
	project := testProject()

	specs, err := ResolveBuildSpecs(project, KindAll, "test-tag")
	if err != nil {
		t.Fatal(err)
	}
	if len(specs) != 4 {
		t.Fatalf("specs len = %d", len(specs))
	}
	kinds := []string{specs[0].Kind, specs[1].Kind, specs[2].Kind, specs[3].Kind}
	if !reflect.DeepEqual(kinds, []string{KindCompanion, KindSlam, KindGazeboSensor, KindOfficialBaseline}) {
		t.Fatalf("kinds = %#v", kinds)
	}
	first := specs[0]
	if first.Image != "world-model/navlab-companion:test-tag" {
		t.Fatalf("image = %q", first.Image)
	}
	wantCommand := []string{
		"docker", "build",
		"-f", "/workspace/docker/Dockerfile.companion",
		"--target", "navlab-companion",
		"-t", "world-model/navlab-companion:test-tag",
		"/workspace",
	}
	if !reflect.DeepEqual(first.Command, wantCommand) {
		t.Fatalf("command = %#v", first.Command)
	}
}

func TestBuildDryRunDoesNotRunDocker(t *testing.T) {
	runner := &recordingRunner{}
	result, err := Build(context.Background(), testProject(), BuildOptions{
		Kind:   KindSlam,
		Tag:    "local",
		DryRun: true,
		Runner: runner,
	})
	if err != nil {
		t.Fatal(err)
	}
	if len(result.Specs) != 1 || result.Specs[0].Kind != KindSlam {
		t.Fatalf("result = %#v", result)
	}
	if len(runner.calls) != 0 {
		t.Fatalf("runner calls = %#v", runner.calls)
	}
}

func TestBuildRunsDockerCommands(t *testing.T) {
	runner := &recordingRunner{}
	var stdout bytes.Buffer
	_, err := Build(context.Background(), testProject(), BuildOptions{
		Kind:   KindGazeboSensor,
		Tag:    "local",
		Runner: runner,
		Stdout: &stdout,
	})
	if err != nil {
		t.Fatal(err)
	}
	if len(runner.calls) != 1 {
		t.Fatalf("runner calls = %#v", runner.calls)
	}
	if !strings.Contains(runner.calls[0], "docker build") || !strings.Contains(runner.calls[0], "navlab-gazebo-sensor:local") {
		t.Fatalf("runner call = %q", runner.calls[0])
	}
	if !strings.Contains(stdout.String(), "Building NavLab gazebo-sensor image") {
		t.Fatalf("stdout = %q", stdout.String())
	}
}

func TestBuildReturnsRunnerError(t *testing.T) {
	runner := &recordingRunner{err: errors.New("docker failed")}
	_, err := Build(context.Background(), testProject(), BuildOptions{
		Kind:   KindCompanion,
		Runner: runner,
	})
	if err == nil || !strings.Contains(err.Error(), "docker failed") {
		t.Fatalf("err = %v", err)
	}
}

func TestResolveBuildSpecsRejectsInvalidKind(t *testing.T) {
	_, err := ResolveBuildSpecs(testProject(), "bad", "")
	if err == nil || !strings.Contains(err.Error(), "invalid image kind") {
		t.Fatalf("err = %v", err)
	}
}

func testProject() config.ProjectConfig {
	return config.ProjectConfig{
		Paths: config.PathConfig{WorkspaceRoot: "/workspace"},
		Navlab: config.NavlabConfig{Images: config.ImageCatalog{
			TagStrategy: "latest",
		}},
		Images: map[string]config.Image{
			"companion": {
				Repository: "world-model/navlab-companion",
				Dockerfile: "docker/Dockerfile.companion",
				Context:    ".",
				Target:     "navlab-companion",
			},
			"slam": {
				Repository: "world-model/navlab-slam-cartographer",
				Dockerfile: "docker/Dockerfile.slam",
				Context:    ".",
				Target:     "navlab-slam-cartographer",
			},
			"gazebo_sensor": {
				Repository: "world-model/navlab-gazebo-sensor",
				Dockerfile: "docker/Dockerfile.gazebo-sensor",
				Context:    ".",
				Target:     "navlab-gazebo-sensor",
			},
			"official_baseline": {
				Repository: "world-model/navlab-official-baseline",
				Dockerfile: "docker/Dockerfile.official-baseline",
				Context:    ".",
				Target:     "navlab-official-baseline",
			},
		},
	}
}
