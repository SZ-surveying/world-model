package helpers

import (
	"context"
	"fmt"
	"os"
	"path/filepath"

	toml "github.com/pelletier/go-toml/v2"
)

const (
	SlamBackendContainer                 = "navlab-slam-backend"
	OfficialExternalNavBridgeParams      = "/opt/navlab_ws/install/navlab_external_nav_bridge/share/navlab_external_nav_bridge/config/navlab_external_nav_bridge.params.yaml"
	OfficialIrisLidarZM                  = "0.075077"
	CartographerOdometryInputTopic       = "/cartographer/odometry_input"
	DiagnosticTruthOdometryTopic         = "/odometry"
	DiagnosticGazeboModelOdometryTopic   = "/gazebo/model/odometry"
	DiagnosticCartographerConfigBasename = "navlab_cartographer_2d_diagnostic_odom.lua"
)

type SlamRuntimeSpec struct {
	Backend                           string
	UseSimTime                        bool
	LaunchPackage                     string
	LaunchFile                        string
	CartographerConfigurationBasename string
	ScanTopic                         string
	IMUTopic                          string
	OdometryTopic                     string
	CartographerTFTopic               string
	PublishGlobalTF                   bool
	GlobalTFTopic                     string
	OdomSourceMode                    string
	SlamOdomTopic                     string
	SlamStatusTopic                   string
	ExternalNavStatusTopic            string
	MapFrameID                        string
	OdomFrameID                       string
	BaseFrameID                       string
	IMUFrameID                        string
	LaserFrameID                      string
}

func ExternalNavBridgeParamsOverride(spec SlamRuntimeSpec) string {
	if spec.Backend == "" {
		spec = DefaultSlamRuntimeSpec()
	}
	return fmt.Sprintf(`navlab_external_nav_bridge_node:
  ros__parameters:
    odom_timeout_ms: 500
    imu_timeout_ms: 500
    height_timeout_ms: 500
    require_imu_for_output: false
    require_height_for_output: true
    input_odom_topic: %s
    imu_topic: %s
    height_topic: /height/estimate
    max_height_covariance: 4.0
    output_odom_topic: /external_nav/odom
    status_topic: %s
    output_frame_id: external_nav
    output_child_frame_id: %s
    ap_tf_topic: /ap/tf
    ap_tf_parent_frame: %s
    ap_tf_child_frame: %s
    expected_odom_frame_id: %s
    expected_odom_child_frame_id: %s
    min_odom_rate_hz: 4.0
    coordinate_mode: pass_through_enu_flu
`,
		spec.SlamOdomTopic,
		spec.IMUTopic,
		spec.ExternalNavStatusTopic,
		spec.BaseFrameID,
		spec.OdomFrameID,
		spec.BaseFrameID,
		spec.MapFrameID,
		spec.BaseFrameID,
	)
}

type SlamContainerSpec struct {
	Image                  string
	RuntimeConfigPath      string
	Backend                string
	SessionID              string
	RosDomainID            string
	RMWImplementation      string
	WorkspaceHostPath      string
	WorkspaceContainerPath string
	User                   string
}

func DefaultSlamRuntimeSpec() SlamRuntimeSpec {
	return SlamRuntimeSpec{
		Backend:                           "cartographer",
		UseSimTime:                        true,
		LaunchPackage:                     "navlab_slam_bringup",
		LaunchFile:                        "navlab_slam_bringup.launch.py",
		CartographerConfigurationBasename: "navlab_cartographer_2d_real.lua",
		ScanTopic:                         "/scan",
		IMUTopic:                          "/imu",
		OdometryTopic:                     CartographerOdometryInputTopic,
		CartographerTFTopic:               "/navlab/slam/tf",
		PublishGlobalTF:                   true,
		GlobalTFTopic:                     "/tf",
		OdomSourceMode:                    "slam_tf",
		SlamOdomTopic:                     "/slam/odom",
		SlamStatusTopic:                   "/navlab/slam/status",
		ExternalNavStatusTopic:            "/external_nav/status",
		MapFrameID:                        "map",
		OdomFrameID:                       "odom",
		BaseFrameID:                       "base_link",
		IMUFrameID:                        "imu_link",
		LaserFrameID:                      "base_scan",
	}
}

func WriteSlamRuntimeConfig(path string, spec SlamRuntimeSpec) error {
	if spec.Backend == "" {
		spec = DefaultSlamRuntimeSpec()
	}
	data := map[string]any{
		"slam": map[string]any{
			"runtime": map[string]any{
				"backend":                                   spec.Backend,
				"use_sim_time":                              spec.UseSimTime,
				"launch_package":                            spec.LaunchPackage,
				"launch_file":                               spec.LaunchFile,
				"launch_fake_odom":                          false,
				"launch_cartographer_backend":               true,
				"publish_placeholder_odom":                  false,
				"cartographer_configuration_basename":       spec.CartographerConfigurationBasename,
				"imu_source_mode":                           "topic",
				"imu_source_topic":                          spec.IMUTopic,
				"imu_source_label":                          "official_gazebo_imu_bridge",
				"imu_min_input_rate_hz":                     "4.0",
				"require_imu_for_external_nav":              false,
				"require_height_for_external_nav":           true,
				"external_nav_input_odom_topic":             spec.SlamOdomTopic,
				"external_nav_expected_odom_frame_id":       spec.MapFrameID,
				"external_nav_expected_odom_child_frame_id": spec.BaseFrameID,
				"external_nav_output_topic":                 "/external_nav/odom",
				"external_nav_status_topic":                 spec.ExternalNavStatusTopic,
				"scan_topic":                                spec.ScanTopic,
				"imu_topic":                                 spec.IMUTopic,
				"cartographer_odometry_topic":               spec.OdometryTopic,
				"cartographer_tf_topic":                     spec.CartographerTFTopic,
				"publish_global_tf":                         spec.PublishGlobalTF,
				"global_tf_topic":                           spec.GlobalTFTopic,
				"cached_odom_publish_rate_hz":               "10.0",
				"odom_source_mode":                          spec.OdomSourceMode,
				"odom_topic":                                spec.SlamOdomTopic,
				"slam_status_topic":                         spec.SlamStatusTopic,
				"map_frame_id":                              spec.MapFrameID,
				"odom_frame_id":                             spec.OdomFrameID,
				"base_frame_id":                             spec.BaseFrameID,
				"imu_frame_id":                              spec.IMUFrameID,
				"laser_frame_id":                            spec.LaserFrameID,
				"base_frame":                                spec.BaseFrameID,
				"imu_frame":                                 spec.IMUFrameID,
				"laser_frame":                               spec.LaserFrameID,
				"laser_x":                                   "0",
				"laser_y":                                   "0",
				"laser_z":                                   OfficialIrisLidarZM,
				"laser_roll":                                "0",
				"laser_pitch":                               "0",
				"laser_yaw":                                 "0",
			},
		},
	}
	encoded, err := toml.Marshal(data)
	if err != nil {
		return err
	}
	if err := os.MkdirAll(filepath.Dir(path), 0o755); err != nil {
		return err
	}
	return os.WriteFile(path, encoded, 0o644)
}

func SlamDockerArgs(spec SlamContainerSpec) ([]string, error) {
	if spec.Image == "" {
		return nil, fmt.Errorf("slam image is required")
	}
	workspaceHostPath := spec.WorkspaceHostPath
	if workspaceHostPath == "" {
		workspaceHostPath = "."
	}
	workspaceContainerPath := spec.WorkspaceContainerPath
	if workspaceContainerPath == "" {
		workspaceContainerPath = "/workspace"
	}
	backend := spec.Backend
	if backend == "" {
		backend = "cartographer"
	}
	command := "source /opt/ros/${ROS_DISTRO:-humble}/setup.bash && " +
		"source /opt/navlab_ws/install/setup.bash && " +
		"exec python3 -m navlab.common.slam.cli launch --config " + shellQuote(spec.RuntimeConfigPath) +
		" --backend " + shellQuote(backend)
	args := []string{
		"run",
		"--detach",
		"--name", SlamBackendContainer,
		"--network", "host",
		"--volume", workspaceHostPath + ":" + workspaceContainerPath,
		"--workdir", workspaceContainerPath,
		"--env", "SESSION_ID=" + spec.SessionID,
		"--env", "ROS_DOMAIN_ID=" + spec.RosDomainID,
		"--env", "RMW_IMPLEMENTATION=" + spec.RMWImplementation,
		"--env", "PYTHONPATH=" + workspaceContainerPath,
		"--env", "NAVLAB_SLAM_RUNTIME_CONFIG=" + spec.RuntimeConfigPath,
	}
	if spec.User != "" {
		args = append(args, "--user", spec.User)
	}
	args = append(args, spec.Image, "bash", "-lc", command)
	return args, nil
}

func StartSlamContainer(ctx context.Context, runner CommandRunner, spec SlamContainerSpec) error {
	if runner == nil {
		runner = ExecCommandRunner{}
	}
	_ = RemoveContainer(ctx, runner, SlamBackendContainer)
	args, err := SlamDockerArgs(spec)
	if err != nil {
		return err
	}
	_, err = runner.Run(ctx, "docker", args...)
	return err
}
