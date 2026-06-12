package helpers

import (
	"context"
	"fmt"
	"os"
	"path/filepath"

	toml "github.com/pelletier/go-toml/v2"
)

const (
	SlamBackendContainer = "navlab-slam-backend"
	OfficialIrisLidarZM  = "0.075077"
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
	SlamOdomTopic                     string
	SlamStatusTopic                   string
	ExternalNavStatusTopic            string
	OdomFrameID                       string
	BaseFrameID                       string
	IMUFrameID                        string
	LaserFrameID                      string
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
		CartographerConfigurationBasename: "navlab_cartographer_2d.lua",
		ScanTopic:                         "/scan",
		IMUTopic:                          "/imu",
		OdometryTopic:                     "/odometry",
		SlamOdomTopic:                     "/slam/odom",
		SlamStatusTopic:                   "/navlab/slam/status",
		ExternalNavStatusTopic:            "/external_nav/status",
		OdomFrameID:                       "odom",
		BaseFrameID:                       "base_link",
		IMUFrameID:                        "imu_link",
		LaserFrameID:                      "laser_frame",
	}
}

func WriteSlamRuntimeConfig(path string, spec SlamRuntimeSpec) error {
	if spec.Backend == "" {
		spec = DefaultSlamRuntimeSpec()
	}
	data := map[string]any{
		"slam": map[string]any{
			"runtime": map[string]any{
				"backend":                             spec.Backend,
				"use_sim_time":                        spec.UseSimTime,
				"launch_package":                      spec.LaunchPackage,
				"launch_file":                         spec.LaunchFile,
				"launch_fake_odom":                    false,
				"launch_cartographer_backend":         true,
				"publish_placeholder_odom":            false,
				"cartographer_configuration_basename": spec.CartographerConfigurationBasename,
				"imu_source_mode":                     "topic",
				"imu_source_topic":                    spec.IMUTopic,
				"imu_source_label":                    "official_gazebo_imu_bridge",
				"imu_min_input_rate_hz":               "4.0",
				"require_imu_for_external_nav":        false,
				"require_height_for_external_nav":     false,
				"external_nav_input_odom_topic":       spec.SlamOdomTopic,
				"external_nav_output_topic":           "/external_nav/odom",
				"external_nav_status_topic":           spec.ExternalNavStatusTopic,
				"scan_topic":                          spec.ScanTopic,
				"imu_topic":                           spec.IMUTopic,
				"cartographer_odometry_topic":         spec.OdometryTopic,
				"odom_topic":                          spec.SlamOdomTopic,
				"slam_status_topic":                   spec.SlamStatusTopic,
				"map_frame_id":                        "map",
				"odom_frame_id":                       spec.OdomFrameID,
				"base_frame_id":                       spec.BaseFrameID,
				"imu_frame_id":                        spec.IMUFrameID,
				"laser_frame_id":                      spec.LaserFrameID,
				"base_frame":                          spec.BaseFrameID,
				"imu_frame":                           spec.IMUFrameID,
				"laser_frame":                         spec.LaserFrameID,
				"laser_x":                             "0",
				"laser_y":                             "0",
				"laser_z":                             OfficialIrisLidarZM,
				"laser_roll":                          "0",
				"laser_pitch":                         "0",
				"laser_yaw":                           "0",
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
	command := "source /opt/ros/jazzy/setup.bash && " +
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
