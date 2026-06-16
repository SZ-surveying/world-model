package helpers

import (
	"context"
	"os"
	"path/filepath"
	"strings"
	"testing"
)

func TestWriteSlamRuntimeConfig(t *testing.T) {
	path := filepath.Join(t.TempDir(), "slam_runtime.toml")
	if err := WriteSlamRuntimeConfig(path, DefaultSlamRuntimeSpec()); err != nil {
		t.Fatalf("WriteSlamRuntimeConfig() error = %v", err)
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
		`laser_frame_id = 'base_scan'`,
		`launch_fake_odom = false`,
		`publish_placeholder_odom = false`,
		`cartographer_configuration_basename = 'navlab_cartographer_2d_real.lua'`,
		`cartographer_odometry_topic = '/cartographer/odometry_input'`,
		`cartographer_tf_topic = '/navlab/slam/tf'`,
		`publish_global_tf = true`,
		`global_tf_topic = '/tf'`,
		`odom_source_mode = 'slam_tf'`,
		`external_nav_input_odom_topic = '/slam/odom'`,
	} {
		if !strings.Contains(text, want) {
			t.Fatalf("runtime config missing %q:\n%s", want, text)
		}
	}
	if strings.Contains(text, `cartographer_odometry_topic = '/odometry'`) {
		t.Fatalf("runtime config must not point Cartographer odometry input at diagnostic truth /odometry:\n%s", text)
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
		"source /opt/ros/${ROS_DISTRO:-humble}/setup.bash",
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
