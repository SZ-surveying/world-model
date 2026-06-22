package helpers

import (
	"context"
	"errors"
	"fmt"
	"os"
	"os/exec"
	"path/filepath"
	"strconv"
	"strings"
)

const (
	GazeboSensorContainer      = "navlab-official-maze-x2-sensor"
	CartographerContainer      = "navlab-official-maze-x2-cartographer"
	OfficialIris3DBridgeConfig = "/opt/navlab_official_ws/install/ardupilot_gz_bringup/share/ardupilot_gz_bringup/config/iris_3Dlidar_bridge.yaml"
)

type CommandRunner interface {
	Run(ctx context.Context, command string, args ...string) (string, error)
}

type ExecCommandRunner struct{}

func (ExecCommandRunner) Run(ctx context.Context, command string, args ...string) (string, error) {
	cmd := exec.CommandContext(ctx, command, args...)
	output, err := cmd.CombinedOutput()
	return string(output), err
}

type GazeboSensorRunSpec struct {
	Image                  string
	SessionID              string
	RosDomainID            string
	RMWImplementation      string
	SensorConfigPath       string
	RuntimeLogPath         string
	WorkspaceHostPath      string
	WorkspaceContainerPath string
}

func WriteBridgeOverride(path string) error {
	return writeText(path, `---
- ros_topic_name: "clock"
  gz_topic_name: "/clock"
  ros_type_name: "rosgraph_msgs/msg/Clock"
  gz_type_name: "gz.msgs.Clock"
  direction: GZ_TO_ROS
- ros_topic_name: "joint_states"
  gz_topic_name: "/world/{{ world_name }}/model/{{ robot_name }}/joint_state"
  ros_type_name: "sensor_msgs/msg/JointState"
  gz_type_name: "gz.msgs.Model"
  direction: GZ_TO_ROS
- ros_topic_name: "gazebo/model/odometry"
  gz_topic_name: "/model/{{ robot_name }}/odometry"
  ros_type_name: "nav_msgs/msg/Odometry"
  gz_type_name: "gz.msgs.Odometry"
  direction: GZ_TO_ROS
- ros_topic_name: "gazebo/tf"
  gz_topic_name: "/model/{{ robot_name }}/pose"
  ros_type_name: "tf2_msgs/msg/TFMessage"
  gz_type_name: "gz.msgs.Pose_V"
  direction: GZ_TO_ROS
- ros_topic_name: "gazebo/tf_static"
  gz_topic_name: "/model/{{ robot_name }}/pose_static"
  ros_type_name: "tf2_msgs/msg/TFMessage"
  gz_type_name: "gz.msgs.Pose_V"
  direction: GZ_TO_ROS
- ros_topic_name: "imu"
  gz_topic_name: "/world/{{ world_name }}/model/{{ robot_name }}/link/imu_link/sensor/imu_sensor/imu"
  ros_type_name: "sensor_msgs/msg/Imu"
  gz_type_name: "gz.msgs.IMU"
  direction: GZ_TO_ROS
- ros_topic_name: "battery"
  gz_topic_name: "/model/{{ robot_name }}/battery/linear_battery/state"
  ros_type_name: "sensor_msgs/msg/BatteryState"
  gz_type_name: "gz.msgs.BatteryState"
  direction: GZ_TO_ROS
- ros_topic_name: "cloud_in"
  gz_topic_name: "/lidar/points"
  ros_type_name: "sensor_msgs/msg/PointCloud2"
  gz_type_name: "gz.msgs.PointCloudPacked"
  direction: GZ_TO_ROS
- ros_topic_name: "rangefinder/down/scan_ideal"
  gz_topic_name: "/rangefinder/down/scan_ideal"
  ros_type_name: "sensor_msgs/msg/LaserScan"
  gz_type_name: "gz.msgs.LaserScan"
  direction: GZ_TO_ROS
`)
}

func WriteVendorProfile(path string, virtualSerialLink string) error {
	return writeText(path, fmt.Sprintf(`ydlidar_ros2_driver_node:
  ros__parameters:
    use_sim_time: true
    port: %s
    frame_id: base_scan
    ignore_array: ""
    baudrate: 115200
    lidar_type: 1
    device_type: 0
    sample_rate: 3
    abnormal_check_count: 4
    fixed_resolution: true
    reversion: false
    inverted: false
    auto_reconnect: true
    isSingleChannel: true
    intensity: false
    support_motor_dtr: true
    angle_max: 180.0
    angle_min: -180.0
    range_max: 8.0
    range_min: 0.1
    frequency: 7.0
`, virtualSerialLink))
}

func WriteModelOverlayFromSource(path string, source string, spec SensorRuntimeSpec) error {
	if !strings.Contains(source, "</model>") {
		return errors.New("official iris_with_lidar model does not contain a closing </model> tag")
	}
	source = strings.Replace(source, "model://lidar_3d", "model://lidar_2d", 1)
	overlay := fmt.Sprintf(`

    <!-- NavLab overlay: down-facing rangefinder ray sensor.
         The X2 vendor-driver input uses the official iris_with_lidar
         /lidar sensor; do not add a second /lidar sensor here. -->
    <link name="rangefinder_down_link">
      <pose relative_to="base_link">0 0 -0.08 0 0 0</pose>
      <inertial>
        <mass>0.03</mass>
        <inertia>
          <ixx>0.00002</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.00002</iyy>
          <iyz>0</iyz>
          <izz>0.00002</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.05 0.04 0.02</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.05 0.04 0.02</size>
          </box>
        </geometry>
      </visual>
      <sensor name="rangefinder_down" type="gpu_lidar">
        <gz_frame_id>%s</gz_frame_id>
        <pose>%s</pose>
        <topic>%s</topic>
        <always_on>true</always_on>
        <update_rate>%g</update_rate>
        <ray>
          <scan>
            <horizontal>
              <samples>%d</samples>
              <resolution>1</resolution>
              <min_angle>0</min_angle>
              <max_angle>0</max_angle>
            </horizontal>
            <vertical>
              <samples>1</samples>
              <resolution>1</resolution>
              <min_angle>0</min_angle>
              <max_angle>0</max_angle>
            </vertical>
          </scan>
          <range>
            <min>%g</min>
            <max>%g</max>
            <resolution>0.005</resolution>
          </range>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>%g</stddev>
          </noise>
        </ray>
        <visualize>false</visualize>
      </sensor>
    </link>

    <joint name="rangefinder_down_joint" type="fixed">
      <parent>base_link</parent>
      <child>rangefinder_down_link</child>
    </joint>
`, spec.RangefinderFrameID, spec.RangefinderModelPose, spec.RangefinderScanIdealTopic, spec.RangefinderModelUpdateHz, spec.RangefinderModelRayCount, spec.RangefinderMinDistanceM, spec.RangefinderMaxDistanceM, spec.RangefinderModelNoiseM)
	rendered := strings.Replace(source, "</model>", overlay+"\n  </model>", 1)
	return writeText(path, rendered)
}

func WriteParamOverlayFromSource(path string, source string, spec SensorRuntimeSpec) error {
	minCM := int(spec.RangefinderMinDistanceM*100 + 0.5)
	maxCM := int(spec.RangefinderMaxDistanceM*100 + 0.5)
	orientation := 25
	overlay := missingParamLines(source, map[string]string{
		"RNGFND1_TYPE":     "20",
		"RNGFND1_ORIENT":   fmt.Sprintf("%d", orientation),
		"RNGFND1_MIN_CM":   fmt.Sprintf("%d", minCM),
		"RNGFND1_MAX_CM":   fmt.Sprintf("%d", maxCM),
		"RNGFND1_GNDCLEAR": "15",
	})
	if overlay == "" {
		return writeText(path, strings.TrimRight(source, "\n")+"\n")
	}
	return writeText(path, strings.TrimRight(source, "\n")+"\n\n# NavLab hardware-faithful down rangefinder overlay.\n"+overlay)
}

func missingParamLines(source string, defaults map[string]string) string {
	seen := map[string]bool{}
	for _, line := range strings.Split(source, "\n") {
		fields := strings.Fields(line)
		if len(fields) == 0 || strings.HasPrefix(fields[0], "#") {
			continue
		}
		seen[fields[0]] = true
	}
	keys := []string{"RNGFND1_TYPE", "RNGFND1_ORIENT", "RNGFND1_MIN_CM", "RNGFND1_MAX_CM", "RNGFND1_GNDCLEAR"}
	lines := []string{}
	for _, key := range keys {
		if seen[key] {
			continue
		}
		lines = append(lines, key+" "+defaults[key])
	}
	if len(lines) == 0 {
		return ""
	}
	return strings.Join(lines, "\n") + "\n"
}

func RemoveContainer(ctx context.Context, runner CommandRunner, name string) error {
	if runner == nil {
		runner = ExecCommandRunner{}
	}
	_, err := runner.Run(ctx, "docker", "rm", "-f", name)
	return err
}

func CaptureContainerLog(ctx context.Context, runner CommandRunner, container string, tail int) (string, error) {
	if runner == nil {
		runner = ExecCommandRunner{}
	}
	if tail <= 0 {
		tail = 2000
	}
	return runner.Run(ctx, "docker", "logs", "--tail", strconv.Itoa(tail), container)
}

func GazeboSensorDockerArgs(spec GazeboSensorRunSpec) ([]string, error) {
	if strings.TrimSpace(spec.Image) == "" {
		return nil, errors.New("gazebo sensor image is required")
	}
	workspaceHostPath := spec.WorkspaceHostPath
	if workspaceHostPath == "" {
		workspaceHostPath = "."
	}
	workspaceContainerPath := spec.WorkspaceContainerPath
	if workspaceContainerPath == "" {
		workspaceContainerPath = "/workspace"
	}
	command := "source /opt/ros/${ROS_DISTRO:-humble}/setup.bash && " +
		"source /opt/navlab_sensor_ws/install/setup.bash && " +
		"exec /opt/gazebo-sensor-venv/bin/python -m navlab.sim.gazebo_sensor.cli --runtime --log-file " +
		shellQuote(spec.RuntimeLogPath)
	args := []string{
		"run",
		"--detach",
		"--name", GazeboSensorContainer,
		"--network", "host",
		"--volume", fmt.Sprintf("%s:%s", workspaceHostPath, workspaceContainerPath),
		"--workdir", workspaceContainerPath,
		"--env", "SESSION_ID=" + spec.SessionID,
		"--env", "ROS_DOMAIN_ID=" + spec.RosDomainID,
		"--env", "RMW_IMPLEMENTATION=" + spec.RMWImplementation,
		"--env", "PYTHONPATH=" + workspaceContainerPath,
		"--env", "NAVLAB_CONFIG=" + spec.SensorConfigPath,
		spec.Image,
		"bash",
		"-lc",
		command,
	}
	return args, nil
}

func StartGazeboSensorContainer(ctx context.Context, runner CommandRunner, spec GazeboSensorRunSpec) error {
	if runner == nil {
		runner = ExecCommandRunner{}
	}
	_ = RemoveContainer(ctx, runner, GazeboSensorContainer)
	args, err := GazeboSensorDockerArgs(spec)
	if err != nil {
		return err
	}
	_, err = runner.Run(ctx, "docker", args...)
	return err
}

func PublisherNodesFromTopicInfo(output string) []string {
	return nodesFromTopicInfoSection(output, "Publisher count:", "Subscription count:")
}

func SubscriptionNodesFromTopicInfo(output string) []string {
	return nodesFromTopicInfoSection(output, "Subscription count:", "")
}

func TopicInfoArtifactName(topic string) string {
	cleaned := strings.Trim(topic, "/")
	if cleaned == "" {
		cleaned = "root"
	}
	cleaned = strings.ReplaceAll(cleaned, "/", "_")
	return "topic_info_" + cleaned + ".txt"
}

func writeText(path string, text string) error {
	if err := os.MkdirAll(filepath.Dir(path), 0o755); err != nil {
		return err
	}
	return os.WriteFile(path, []byte(text), 0o644)
}

func nodesFromTopicInfoSection(output string, startMarker string, stopMarker string) []string {
	var nodes []string
	inSection := false
	for _, line := range strings.Split(output, "\n") {
		stripped := strings.TrimSpace(line)
		if strings.HasPrefix(stripped, startMarker) {
			inSection = true
			continue
		}
		if stopMarker != "" && strings.HasPrefix(stripped, stopMarker) {
			inSection = false
			continue
		}
		if inSection && strings.HasPrefix(stripped, "Node name:") {
			parts := strings.SplitN(stripped, ":", 2)
			if len(parts) == 2 {
				nodes = append(nodes, strings.TrimSpace(parts[1]))
			}
		}
	}
	return nodes
}

func shellQuote(value string) string {
	if value == "" {
		return "''"
	}
	return "'" + strings.ReplaceAll(value, "'", "'\"'\"'") + "'"
}
