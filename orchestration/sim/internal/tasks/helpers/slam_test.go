package helpers

import (
	"context"
	"os"
	"path/filepath"
	"strings"
	"testing"
)

func TestWriteP3SlamRuntimeConfig(t *testing.T) {
	path := filepath.Join(t.TempDir(), "slam_runtime.toml")
	if err := WriteP3SlamRuntimeConfig(path, DefaultSlamRuntimeSpec()); err != nil {
		t.Fatalf("WriteP3SlamRuntimeConfig() error = %v", err)
	}
	data, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	text := string(data)
	for _, want := range []string{
		`backend = 'cartographer'`,
		`scan_topic = '/scan'`,
		`laser_z = '0.075077'`,
	} {
		if !strings.Contains(text, want) {
			t.Fatalf("runtime config missing %q:\n%s", want, text)
		}
	}
}

func TestSlamDockerArgs(t *testing.T) {
	args, err := SlamDockerArgs(SlamContainerSpec{
		Image:             "slam-image",
		RuntimeConfigPath: "/workspace/artifacts/slam_runtime.toml",
		Backend:           "cartographer",
		RosDomainID:       "85",
	})
	if err != nil {
		t.Fatalf("SlamDockerArgs() error = %v", err)
	}
	joined := strings.Join(args, " ")
	for _, want := range []string{
		"--name " + SlamBackendContainer,
		"NAVLAB_SLAM_RUNTIME_CONFIG=/workspace/artifacts/slam_runtime.toml",
		"navlab.common.slam.cli launch",
		"--backend 'cartographer'",
	} {
		if !strings.Contains(joined, want) {
			t.Fatalf("docker args missing %q:\n%s", want, joined)
		}
	}
}

func TestStartSlamContainerUsesDockerRunner(t *testing.T) {
	runner := &fakeRunner{}
	err := StartSlamContainer(context.Background(), runner, SlamContainerSpec{
		Image:             "slam-image",
		RuntimeConfigPath: "/workspace/slam.toml",
	})
	if err != nil {
		t.Fatalf("StartSlamContainer() error = %v", err)
	}
	if len(runner.calls) != 2 {
		t.Fatalf("calls = %#v, want remove + run", runner.calls)
	}
	if !strings.Contains(runner.calls[0], "docker rm -f "+SlamBackendContainer) {
		t.Fatalf("first call = %q", runner.calls[0])
	}
	if !strings.Contains(runner.calls[1], "docker run") {
		t.Fatalf("second call = %q", runner.calls[1])
	}
}
