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
		`imu_source_topic = '/imu'`,
		`imu_topic = '/imu'`,
		`laser_z = '0.075077'`,
		`laser_frame_id = 'base_scan'`,
		`launch_fake_odom = false`,
		`launch_cartographer_backend = true`,
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

func TestHoverSlamRuntimeSpecUsesHoverConfigAndNonTruthOdom(t *testing.T) {
	spec := HoverSlamRuntimeSpec(DefaultSlamRuntimeSpec())
	if spec.CartographerConfigurationBasename != HoverCartographerConfigBasename {
		t.Fatalf("hover Cartographer config = %q", spec.CartographerConfigurationBasename)
	}
	if spec.OdometryTopic != CartographerOdometryInputTopic {
		t.Fatalf("hover odometry topic = %q", spec.OdometryTopic)
	}
	if spec.OdometryTopic == DiagnosticTruthOdometryTopic {
		t.Fatalf("hover odometry topic must not use diagnostic truth %q", DiagnosticTruthOdometryTopic)
	}
	if spec.IMUSourceTopic != "/imu" || spec.IMUTopic != "/navlab/slam/imu" {
		t.Fatalf("hover IMU topics source=%q output=%q", spec.IMUSourceTopic, spec.IMUTopic)
	}
}

func TestDefaultCartographerScanReferenceOdometrySpecUsesScanOnlyOdomInput(t *testing.T) {
	spec := DefaultCartographerScanReferenceOdometrySpec()
	if spec.OdomTopic != CartographerOdometryInputTopic {
		t.Fatalf("odom topic = %q", spec.OdomTopic)
	}
	if spec.OdomTopic == DiagnosticTruthOdometryTopic {
		t.Fatalf("scan-reference odom prior must not use diagnostic truth %q", DiagnosticTruthOdometryTopic)
	}
	if spec.FrameID != "odom" || spec.ChildFrameID != "base_link" {
		t.Fatalf("frames = %q -> %q", spec.FrameID, spec.ChildFrameID)
	}
	if spec.ResetOnHoverHold {
		t.Fatalf("Cartographer odometry input must not reset on hover_hold")
	}
}

func TestWriteSlamRuntimeConfigCanSplitSlamOdomFromExternalNavInput(t *testing.T) {
	path := filepath.Join(t.TempDir(), "slam_runtime.toml")
	spec := DefaultSlamRuntimeSpec()
	spec.SlamOdomTopic = "/slam/cartographer_odom"
	spec.ExternalNavInputOdomTopic = "/slam/odom"
	if err := WriteSlamRuntimeConfig(path, spec); err != nil {
		t.Fatalf("WriteSlamRuntimeConfig() error = %v", err)
	}
	data, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	text := string(data)
	for _, want := range []string{
		`odom_topic = '/slam/cartographer_odom'`,
		`external_nav_input_odom_topic = '/slam/odom'`,
	} {
		if !strings.Contains(text, want) {
			t.Fatalf("runtime config missing %q:\n%s", want, text)
		}
	}
	if strings.Contains(text, `external_nav_input_odom_topic = '/slam/cartographer_odom'`) {
		t.Fatalf("external nav input must not silently follow side-channel odom:\n%s", text)
	}
}

func TestExternalNavBridgeParamsExposeSlamQualityGate(t *testing.T) {
	spec := DefaultSlamRuntimeSpec()
	spec.IMUTopic = "/navlab/slam/imu"
	spec.RequireIMUForQuality = true
	spec.RequireScanForQuality = true
	spec.LowObservabilityMode = true

	text := ExternalNavBridgeParamsOverride(spec)

	for _, want := range []string{
		"slam_quality_gate_enabled: true",
		"require_imu_for_quality: true",
		"require_scan_for_quality: true",
		"low_observability_mode: true",
		"scan_topic: /scan",
		"min_imu_rate_hz: 4.0",
		"min_scan_rate_hz: 2.0",
		"max_position_jump_m: 0.75",
		"max_yaw_jump_rad: 0.75",
		"min_scan_valid_ratio_for_quality: 0.50",
		"min_scan_hit_ratio_for_quality: 0.25",
		"min_scan_range_span_m_for_quality: 1.0",
		"min_scan_range_stddev_m_for_quality: 0.20",
		"min_scan_observed_quadrants_for_quality: 3",
	} {
		if !strings.Contains(text, want) {
			t.Fatalf("external nav params missing %q:\n%s", want, text)
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
