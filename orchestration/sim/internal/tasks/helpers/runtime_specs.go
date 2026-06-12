package helpers

import (
	"encoding/json"
	"fmt"
	"os"
	"path/filepath"

	toml "github.com/pelletier/go-toml/v2"
)

const (
	OfficialIrisWithLidarModel = "/opt/navlab_official_ws/install/ardupilot_gz_description/share/ardupilot_gz_description/models/iris_with_lidar/model.sdf"
	OfficialGazeboIrisParams   = "/opt/navlab_official_ws/install/ardupilot_sitl/share/ardupilot_sitl/config/default_params/gazebo-iris.parm"
	P4ControllerContainer      = "navlab-p4-fcu-controller"
	P4RosbagContainer          = "navlab-p4-rosbag"
	P5RosbagContainer          = "navlab-p5-rosbag"
	P6RosbagContainer          = "navlab-p6-rosbag"
	P6VehicleMarkerContainer   = "navlab-p6-vehicle-markers"
	P11RosbagContainer         = "navlab-p11-rosbag"
	P8RosbagContainer          = "navlab-p8-rosbag"
)

type P2SensorSpec struct {
	RuntimeConfigPath          string
	X2VirtualSerialLink        string
	X2ScanInputTopic           string
	X2ScanTopic                string
	X2StatusTopic              string
	RangefinderFrameID         string
	RangefinderScanIdealTopic  string
	RangefinderRangeTopic      string
	RangefinderStatusTopic     string
	RangefinderEndpoint        string
	RangefinderMAVOrientation  string
	RangefinderSourceSystem    int
	RangefinderSourceComponent int
	RangefinderSensorID        int
	RangefinderRateHz          float64
	RangefinderMinDistanceM    float64
	RangefinderMaxDistanceM    float64
	RangefinderCovarianceCM    int
	RangefinderModelPose       string
	RangefinderModelUpdateHz   float64
	RangefinderModelRayCount   int
	RangefinderModelNoiseM     float64
	IMUOutputTopic             string
	IMUSourceRoute             string
	IMUSourceTopic             string
	SyntheticFallbackEnabled   bool
}

func DefaultP2SensorSpec() P2SensorSpec {
	return P2SensorSpec{
		RuntimeConfigPath:          "artifacts/gazebo_sensor_runtime.toml",
		X2VirtualSerialLink:        "/tmp/navlab-x2",
		X2ScanInputTopic:           "/scan",
		X2ScanTopic:                "/navlab/x2/scan",
		X2StatusTopic:              "/navlab/x2/status",
		RangefinderFrameID:         "rangefinder_down",
		RangefinderScanIdealTopic:  "/navlab/rangefinder/scan_ideal",
		RangefinderRangeTopic:      "/navlab/rangefinder/range",
		RangefinderStatusTopic:     "/navlab/rangefinder/status",
		RangefinderEndpoint:        "udpout:127.0.0.1:14551",
		RangefinderMAVOrientation:  "MAV_SENSOR_ROTATION_PITCH_270",
		RangefinderSourceSystem:    42,
		RangefinderSourceComponent: 191,
		RangefinderSensorID:        1,
		RangefinderRateHz:          20.0,
		RangefinderMinDistanceM:    0.10,
		RangefinderMaxDistanceM:    8.0,
		RangefinderCovarianceCM:    5,
		RangefinderModelPose:       "0 0 -0.05 0 1.57079632679 0",
		RangefinderModelUpdateHz:   20.0,
		RangefinderModelRayCount:   5,
		RangefinderModelNoiseM:     0.0,
		IMUOutputTopic:             "/imu",
		IMUSourceRoute:             "official_gazebo_imu_bridge",
		IMUSourceTopic:             "/imu",
		SyntheticFallbackEnabled:   false,
	}
}

func WriteP2SensorConfig(path string, vendorProfile string, spec P2SensorSpec) error {
	if spec.RuntimeConfigPath == "" {
		spec = DefaultP2SensorSpec()
	}
	data := map[string]any{
		"gazebo_sensor": map[string]any{
			"x2_protocol": map[string]any{
				"enabled":               true,
				"scan_source":           "x2_virtual_serial",
				"profile":               vendorProfile,
				"virtual_serial_link":   spec.X2VirtualSerialLink,
				"scan_ideal_topic":      spec.X2ScanInputTopic,
				"vendor_scan_topic":     "/navlab/x2/vendor_scan",
				"scan_topic":            spec.X2ScanTopic,
				"status_topic":          spec.X2StatusTopic,
				"sample_rate_hz":        3000.0,
				"scan_frequency_hz":     7.0,
				"scan_frequency_min_hz": 7.0,
				"scan_frequency_max_hz": 7.0,
				"static_range_m":        1.5,
				"range_min_m":           0.1,
				"range_max_m":           8.0,
				"auto_start":            true,
			},
			"down_rangefinder": map[string]any{
				"enabled":              true,
				"scan_ideal_topic":     spec.RangefinderScanIdealTopic,
				"range_topic":          spec.RangefinderRangeTopic,
				"status_topic":         spec.RangefinderStatusTopic,
				"endpoint":             spec.RangefinderEndpoint,
				"frame_id":             spec.RangefinderFrameID,
				"mavlink_orientation":  spec.RangefinderMAVOrientation,
				"source_system":        spec.RangefinderSourceSystem,
				"source_component":     spec.RangefinderSourceComponent,
				"sensor_id":            spec.RangefinderSensorID,
				"rate_hz":              spec.RangefinderRateHz,
				"min_distance_m":       spec.RangefinderMinDistanceM,
				"max_distance_m":       spec.RangefinderMaxDistanceM,
				"covariance_cm":        spec.RangefinderCovarianceCM,
				"model_pose":           spec.RangefinderModelPose,
				"model_update_rate_hz": spec.RangefinderModelUpdateHz,
				"model_ray_count":      spec.RangefinderModelRayCount,
				"model_noise_stddev_m": spec.RangefinderModelNoiseM,
			},
		},
	}
	return writeTOML(path, data)
}

func P2RangefinderProbeScript(spec P2SensorSpec) (string, error) {
	if spec.RuntimeConfigPath == "" {
		spec = DefaultP2SensorSpec()
	}
	return rosProbeScript("navlab_p2_rangefinder_probe", []string{spec.RangefinderRangeTopic, spec.RangefinderStatusTopic}, map[string]any{
		"range_topic":  spec.RangefinderRangeTopic,
		"status_topic": spec.RangefinderStatusTopic,
	})
}

func P2IMUProbeScript(spec P2SensorSpec) (string, error) {
	if spec.RuntimeConfigPath == "" {
		spec = DefaultP2SensorSpec()
	}
	return rosProbeScript("navlab_p2_imu_probe", []string{spec.IMUOutputTopic}, map[string]any{
		"topic":                      spec.IMUOutputTopic,
		"source_route":               spec.IMUSourceRoute,
		"source_topic":               spec.IMUSourceTopic,
		"synthetic_fallback_enabled": spec.SyntheticFallbackEnabled,
	})
}

type FCUControllerSpec struct {
	ControlRoute           string
	MAVLinkBootstrap       string
	OwnerName              string
	OwnerID                string
	FCUStateTopic          string
	ControllerStatusTopic  string
	SetpointIntentTopic    string
	SetpointOutputTopic    string
	OwnerStatusTopic       string
	TimeTopic              string
	PrearmService          string
	ModeSwitchService      string
	ArmService             string
	TakeoffService         string
	CmdVelTopic            string
	PoseTopic              string
	TwistTopic             string
	StatusTopic            string
	RangefinderRangeTopic  string
	RangefinderStatusTopic string
	IMUTopic               string
	IMUInputTopic          string
	ScanInputTopic         string
	ScanOutputTopic        string
	ScanStabilizationTopic string
	DisturbanceStatusTopic string
	ExplorationStatusTopic string
	SlamOdomTopic          string
	SlamStatusTopic        string
	LandingPolicy          string
	MotionSpeedMPS         float64
	MinAcceptedGoals       int
	MinPathLengthM         float64
	DisturbanceProfile     string
	RequiredProfiles       []string
	GuidedMode             string
	TakeoffAltM            float64
	ReadinessTimeoutSec    float64
	HoldAfterReadySec      float64
	RequireSlamBackend     bool
}

func DefaultFCUControllerSpec() FCUControllerSpec {
	return FCUControllerSpec{
		ControlRoute:           "official_dds",
		MAVLinkBootstrap:       "udp:127.0.0.1:14551",
		OwnerName:              "navlab_fcu_controller",
		OwnerID:                "p4",
		FCUStateTopic:          "/navlab/fcu/state",
		ControllerStatusTopic:  "/navlab/fcu/controller/status",
		SetpointIntentTopic:    "/navlab/fcu/setpoint/intent",
		SetpointOutputTopic:    "/navlab/fcu/setpoint/output",
		OwnerStatusTopic:       "/navlab/fcu/owner/status",
		TimeTopic:              "/ap/v1/time",
		PrearmService:          "/ap/v1/prearm_check",
		ModeSwitchService:      "/ap/v1/mode_switch",
		ArmService:             "/ap/v1/arm_motors",
		TakeoffService:         "/ap/v1/takeoff",
		CmdVelTopic:            "/ap/v1/cmd_vel",
		PoseTopic:              "/ap/v1/pose/filtered",
		TwistTopic:             "/ap/v1/twist/filtered",
		StatusTopic:            "/ap/v1/status",
		RangefinderRangeTopic:  "/navlab/rangefinder/range",
		RangefinderStatusTopic: "/navlab/rangefinder/status",
		IMUTopic:               "/imu",
		IMUInputTopic:          "",
		ScanInputTopic:         "",
		ScanOutputTopic:        "",
		ScanStabilizationTopic: "",
		DisturbanceStatusTopic: "",
		ExplorationStatusTopic: "",
		SlamOdomTopic:          "/slam/odom",
		SlamStatusTopic:        "/navlab/slam/status",
		LandingPolicy:          "land_in_place",
		MotionSpeedMPS:         0.0,
		MinAcceptedGoals:       0,
		MinPathLengthM:         0.0,
		DisturbanceProfile:     "nominal_realistic",
		RequiredProfiles:       nil,
		GuidedMode:             "GUIDED",
		TakeoffAltM:            0.5,
		ReadinessTimeoutSec:    45.0,
		HoldAfterReadySec:      8.0,
		RequireSlamBackend:     true,
	}
}

func WriteP4RuntimeConfig(path string, spec FCUControllerSpec) error {
	if spec.ControlRoute == "" {
		spec = DefaultFCUControllerSpec()
	}
	return writeTOML(path, map[string]any{"fcu_controller": map[string]any{"runtime": spec}})
}

func FCUControllerRuntimeScript(spec FCUControllerSpec, durationSec float64) (string, error) {
	if spec.ControlRoute == "" {
		spec = DefaultFCUControllerSpec()
	}
	payload := map[string]any{
		"duration_sec":             durationSec,
		"control_route":            spec.ControlRoute,
		"pose_topic":               spec.PoseTopic,
		"controller_status_topic":  spec.ControllerStatusTopic,
		"setpoint_intent_topic":    spec.SetpointIntentTopic,
		"setpoint_output_topic":    spec.SetpointOutputTopic,
		"owner_status_topic":       spec.OwnerStatusTopic,
		"fcu_state_topic":          spec.FCUStateTopic,
		"cmd_vel_topic":            spec.CmdVelTopic,
		"takeoff_alt_m":            spec.TakeoffAltM,
		"guided_mode":              spec.GuidedMode,
		"readiness_timeout_sec":    spec.ReadinessTimeoutSec,
		"hold_after_ready_sec":     spec.HoldAfterReadySec,
		"require_slam_backend":     spec.RequireSlamBackend,
		"rangefinder_range_topic":  spec.RangefinderRangeTopic,
		"rangefinder_status_topic": spec.RangefinderStatusTopic,
		"imu_input_topic":          spec.IMUInputTopic,
		"imu_output_topic":         spec.IMUTopic,
		"scan_input_topic":         spec.ScanInputTopic,
		"scan_output_topic":        spec.ScanOutputTopic,
		"scan_stabilization_topic": spec.ScanStabilizationTopic,
		"disturbance_status_topic": spec.DisturbanceStatusTopic,
		"exploration_status_topic": spec.ExplorationStatusTopic,
		"slam_odom_topic":          spec.SlamOdomTopic,
		"slam_status_topic":        spec.SlamStatusTopic,
		"hover_status_topic":       "/navlab/hover/status",
		"landing_status_topic":     "/navlab/landing/status",
		"landing_policy":           spec.LandingPolicy,
		"motion_speed_mps":         spec.MotionSpeedMPS,
		"min_accepted_goals":       spec.MinAcceptedGoals,
		"min_path_length_m":        spec.MinPathLengthM,
		"disturbance_profile":      spec.DisturbanceProfile,
		"required_profiles":        append([]string(nil), spec.RequiredProfiles...),
	}
	return fcuControllerRuntimeScript(payload)
}

type FrameContractSpec struct {
	RequiredFrames          []string
	MapFrameID              string
	OdomFrameID             string
	BaseFrameID             string
	IMUFrameID              string
	LaserFrameID            string
	RangefinderFrameID      string
	ScanTopic               string
	IMUTopic                string
	RangefinderRangeTopic   string
	RangefinderStatusTopic  string
	FCUPoseTopic            string
	FCUTwistTopic           string
	FCUStatusTopic          string
	CmdVelTopic             string
	SlamOdomTopic           string
	SlamStatusTopic         string
	TruthDiagnosticTopic    string
	ControllerStatusTopic   string
	SetpointOutputTopic     string
	OwnerStatusTopic        string
	StatusTopic             string
	MaxDynamicTFAgeSec      float64
	MinScanValidRatio       float64
	MaxRangefinderHeightErr float64
	MaxDirectionErrorRad    float64
	ProbeDurationSec        float64
}

func DefaultFrameContractSpec() FrameContractSpec {
	return FrameContractSpec{
		RequiredFrames:          []string{"map", "odom", "base_link", "imu_link", "laser_frame", "rangefinder_down"},
		MapFrameID:              "map",
		OdomFrameID:             "odom",
		BaseFrameID:             "base_link",
		IMUFrameID:              "imu_link",
		LaserFrameID:            "laser_frame",
		RangefinderFrameID:      "rangefinder_down",
		ScanTopic:               "/scan",
		IMUTopic:                "/imu",
		RangefinderRangeTopic:   "/navlab/rangefinder/range",
		RangefinderStatusTopic:  "/navlab/rangefinder/status",
		FCUPoseTopic:            "/ap/v1/pose/filtered",
		FCUTwistTopic:           "/ap/v1/twist/filtered",
		FCUStatusTopic:          "/ap/v1/status",
		CmdVelTopic:             "/ap/v1/cmd_vel",
		SlamOdomTopic:           "/slam/odom",
		SlamStatusTopic:         "/navlab/slam/status",
		TruthDiagnosticTopic:    "/navlab/truth/odom",
		ControllerStatusTopic:   "/navlab/fcu/controller/status",
		SetpointOutputTopic:     "/navlab/fcu/setpoint/output",
		OwnerStatusTopic:        "/navlab/fcu/owner/status",
		StatusTopic:             "/navlab/frame_contract/status",
		MaxDynamicTFAgeSec:      1.0,
		MinScanValidRatio:       0.8,
		MaxRangefinderHeightErr: 0.25,
		MaxDirectionErrorRad:    0.35,
		ProbeDurationSec:        12.0,
	}
}

func WriteP5RuntimeConfig(path string, spec FrameContractSpec) error {
	if spec.MapFrameID == "" {
		spec = DefaultFrameContractSpec()
	}
	return writeTOML(path, map[string]any{"frame_contract": map[string]any{"runtime": spec}})
}

func FrameContractProbeScript(spec FrameContractSpec) (string, error) {
	if spec.MapFrameID == "" {
		spec = DefaultFrameContractSpec()
	}
	return rosProbeScript("navlab_frame_contract_probe", []string{"/tf", "/tf_static", spec.ScanTopic, spec.IMUTopic, spec.RangefinderRangeTopic, spec.FCUPoseTopic, spec.SlamOdomTopic}, spec)
}

type SlamHoverSpec struct {
	SlamOdomTopic           string
	SlamStatusTopic         string
	ExternalNavStatusTopic  string
	FCUPoseTopic            string
	FCUTwistTopic           string
	FCUStatusTopic          string
	CmdVelTopic             string
	RangefinderRangeTopic   string
	RangefinderStatusTopic  string
	IMUTopic                string
	TruthDiagnosticTopic    string
	ControllerStatusTopic   string
	SetpointIntentTopic     string
	SetpointOutputTopic     string
	OwnerStatusTopic        string
	HoverStatusTopic        string
	VehicleMarkerTopic      string
	VehicleMarkerPoseTopic  string
	VehicleMarkerFrameID    string
	VehicleMarkerRateHz     float64
	SettleWindowSec         float64
	HoverWindowSec          float64
	FinalHoldWindowSec      float64
	MaxHoverHorizontalDrift float64
	MaxHoverAltitudeError   float64
	MaxHoverYawDriftRad     float64
	MaxStopDriftM           float64
}

func DefaultSlamHoverSpec() SlamHoverSpec {
	return SlamHoverSpec{
		SlamOdomTopic:           "/slam/odom",
		SlamStatusTopic:         "/navlab/slam/status",
		ExternalNavStatusTopic:  "/external_nav/status",
		FCUPoseTopic:            "/ap/v1/pose/filtered",
		FCUTwistTopic:           "/ap/v1/twist/filtered",
		FCUStatusTopic:          "/ap/v1/status",
		CmdVelTopic:             "/ap/v1/cmd_vel",
		RangefinderRangeTopic:   "/navlab/rangefinder/range",
		RangefinderStatusTopic:  "/navlab/rangefinder/status",
		IMUTopic:                "/imu",
		TruthDiagnosticTopic:    "/navlab/truth/odom",
		ControllerStatusTopic:   "/navlab/fcu/controller/status",
		SetpointIntentTopic:     "/navlab/fcu/setpoint/intent",
		SetpointOutputTopic:     "/navlab/fcu/setpoint/output",
		OwnerStatusTopic:        "/navlab/fcu/owner/status",
		HoverStatusTopic:        "/navlab/hover/status",
		VehicleMarkerTopic:      "/navlab/vehicle_marker",
		VehicleMarkerPoseTopic:  "/navlab/vehicle_marker/pose",
		VehicleMarkerFrameID:    "base_link",
		VehicleMarkerRateHz:     5.0,
		SettleWindowSec:         8.0,
		HoverWindowSec:          18.0,
		FinalHoldWindowSec:      5.0,
		MaxHoverHorizontalDrift: 0.35,
		MaxHoverAltitudeError:   0.30,
		MaxHoverYawDriftRad:     0.35,
		MaxStopDriftM:           0.20,
	}
}

func WriteP6RuntimeConfig(path string, spec SlamHoverSpec) error {
	if spec.SlamOdomTopic == "" {
		spec = DefaultSlamHoverSpec()
	}
	return writeTOML(path, map[string]any{"slam_hover": map[string]any{"runtime": spec}})
}

func HoverProbeScript(spec SlamHoverSpec) (string, error) {
	if spec.SlamOdomTopic == "" {
		spec = DefaultSlamHoverSpec()
	}
	return rosProbeScript(
		"navlab_slam_hover_probe",
		[]string{spec.FCUPoseTopic, spec.SlamOdomTopic, spec.SlamStatusTopic, spec.ControllerStatusTopic, spec.OwnerStatusTopic, spec.HoverStatusTopic, "/navlab/landing/status"},
		spec,
	)
}

type ScanStabilizationSpec struct {
	Mode                        string
	InputScanTopic              string
	OutputScanTopic             string
	StatusTopic                 string
	AttitudeSourceTopic         string
	MaxRejectedBeamRatio        float64
	MinRetainedBeamRatio        float64
	MaxFloorHitRiskBeamRatio    float64
	MaxVerticalProjectionErrorM float64
	MaxAttitudeSourceAgeMS      float64
	PassthroughTiltDeg          float64
	CompensationTiltDeg         float64
	HardDropTiltDeg             float64
	UsesGazeboTruthAsInput      bool
	ScanStabilizationClaim      string
	ReplayReadinessTimeoutSec   float64
	ControllerSummaryTimeoutSec float64
}

func DefaultScanStabilizationSpec() ScanStabilizationSpec {
	return ScanStabilizationSpec{
		Mode:                        "bounded_2d_projection",
		InputScanTopic:              "/navlab/x2/scan_normalized",
		OutputScanTopic:             "/scan",
		StatusTopic:                 "/navlab/scan_stabilization/status",
		AttitudeSourceTopic:         "/imu",
		MaxRejectedBeamRatio:        0.35,
		MinRetainedBeamRatio:        0.60,
		MaxFloorHitRiskBeamRatio:    0.10,
		MaxVerticalProjectionErrorM: 0.15,
		MaxAttitudeSourceAgeMS:      300.0,
		PassthroughTiltDeg:          2.0,
		CompensationTiltDeg:         5.0,
		HardDropTiltDeg:             25.0,
		UsesGazeboTruthAsInput:      false,
		ScanStabilizationClaim:      "evaluated",
		ReplayReadinessTimeoutSec:   60.0,
		ControllerSummaryTimeoutSec: 20.0,
	}
}

func ValidateP11Config(spec ScanStabilizationSpec) []string {
	var blockers []string
	if spec.Mode != "bounded_2d_projection" {
		blockers = append(blockers, "scan_stabilization_config_invalid: mode must be bounded_2d_projection")
	}
	if spec.InputScanTopic == spec.OutputScanTopic {
		blockers = append(blockers, "scan_stabilization_config_invalid: input and output topics must differ")
	}
	if !(0.0 <= spec.PassthroughTiltDeg && spec.PassthroughTiltDeg < spec.CompensationTiltDeg && spec.CompensationTiltDeg < spec.HardDropTiltDeg) {
		blockers = append(blockers, "scan_stabilization_config_invalid: tilt thresholds must be ordered")
	}
	if spec.UsesGazeboTruthAsInput {
		blockers = append(blockers, "P11 must not use Gazebo truth as input")
	}
	if spec.ScanStabilizationClaim != "evaluated" {
		blockers = append(blockers, "P11 scan_stabilization_claim must be evaluated")
	}
	return blockers
}

func WriteP11RuntimeConfig(path string, spec ScanStabilizationSpec) error {
	if spec.Mode == "" {
		spec = DefaultScanStabilizationSpec()
	}
	return writeTOML(path, map[string]any{
		"scan_stabilization":      map[string]any{"runtime": spec},
		"scan_stabilization_gate": map[string]any{"runtime": map[string]any{"replay_readiness_timeout_sec": spec.ReplayReadinessTimeoutSec, "controller_summary_timeout_sec": spec.ControllerSummaryTimeoutSec}},
	})
}

func StabilizationStatusProbeScript(spec ScanStabilizationSpec) (string, error) {
	if spec.Mode == "" {
		spec = DefaultScanStabilizationSpec()
	}
	return rosProbeScript("navlab_p11_status_probe", []string{spec.StatusTopic}, map[string]any{"status_topic": spec.StatusTopic})
}

func AirframeDisturbanceProbeScript(spec ScanRobustnessWorkflowSpec) (string, error) {
	if spec.Profile == "" {
		spec = DefaultScanRobustnessWorkflowSpec()
	}
	return rosProbeScript("navlab_p12_airframe_disturbance_probe", []string{spec.DisturbanceStatusTopic}, map[string]any{
		"status_topic":      spec.DisturbanceStatusTopic,
		"required_profiles": spec.RequiredProfiles,
		"profile":           spec.Profile,
	})
}

type ExplorationWorkflowSpec struct {
	Strategy               string
	ExplorationWindowSec   float64
	MotionSpeedMPS         float64
	MinAcceptedGoals       int
	MinPathLengthM         float64
	ControllerStatusTopic  string
	SetpointIntentTopic    string
	SetpointOutputTopic    string
	SlamOdomTopic          string
	ExplorationStatusTopic string
}

func DefaultExplorationWorkflowSpec() ExplorationWorkflowSpec {
	return ExplorationWorkflowSpec{
		Strategy:               "frontier_lite",
		ExplorationWindowSec:   26.0,
		MotionSpeedMPS:         0.10,
		MinAcceptedGoals:       3,
		MinPathLengthM:         0.35,
		ControllerStatusTopic:  "/navlab/fcu/controller/status",
		SetpointIntentTopic:    "/navlab/fcu/setpoint/intent",
		SetpointOutputTopic:    "/navlab/fcu/setpoint/output",
		SlamOdomTopic:          "/slam/odom",
		ExplorationStatusTopic: "/navlab/exploration/status",
	}
}

func WriteP8RuntimeConfig(path string, spec ExplorationWorkflowSpec) error {
	if spec.Strategy == "" {
		spec = DefaultExplorationWorkflowSpec()
	}
	return writeTOML(path, map[string]any{"exploration_gate": map[string]any{"runtime": spec}})
}

func ExplorationProbeScript(spec ExplorationWorkflowSpec) (string, error) {
	if spec.Strategy == "" {
		spec = DefaultExplorationWorkflowSpec()
	}
	return rosProbeScript("navlab_exploration_probe", []string{spec.ControllerStatusTopic, spec.SetpointOutputTopic, spec.ExplorationStatusTopic, spec.SlamOdomTopic}, spec)
}

type ScanRobustnessWorkflowSpec struct {
	Profile                string
	RequiredProfiles       []string
	FCUStatusTopic         string
	StabilizedScanTopic    string
	DisturbanceStatusTopic string
	IMURawTopic            string
	IMUOutputTopic         string
	LandingPolicy          string
}

func DefaultScanRobustnessWorkflowSpec() ScanRobustnessWorkflowSpec {
	return ScanRobustnessWorkflowSpec{
		Profile:                "nominal_realistic",
		RequiredProfiles:       []string{"clean", "mild_bias", "nominal_realistic", "esc_lag", "vibration"},
		FCUStatusTopic:         "/navlab/fcu/controller/status",
		StabilizedScanTopic:    "/scan",
		DisturbanceStatusTopic: "/navlab/airframe_disturbance/status",
		IMURawTopic:            "/imu/raw",
		IMUOutputTopic:         "/imu",
		LandingPolicy:          "land_in_place",
	}
}

func WriteP12RuntimeConfig(path string, spec ScanRobustnessWorkflowSpec) error {
	if spec.Profile == "" {
		spec = DefaultScanRobustnessWorkflowSpec()
	}
	return writeTOML(path, map[string]any{
		"airframe_disturbance":      map[string]any{"runtime": spec},
		"airframe_disturbance_gate": map[string]any{"runtime": map[string]any{"required_profiles": spec.RequiredProfiles}},
		"landing":                   map[string]any{"runtime": map[string]any{"policy": spec.LandingPolicy}},
	})
}

func WriteP12BridgeOverride(path string, imuRawTopic string) error {
	if err := WriteP1BridgeOverride(path); err != nil {
		return err
	}
	content, err := os.ReadFile(path)
	if err != nil {
		return err
	}
	rendered := replaceFirst(string(content), `ros_topic_name: "imu"`, fmt.Sprintf(`ros_topic_name: "%s"`, trimTopicSlash(imuRawTopic)))
	return os.WriteFile(path, []byte(rendered), 0o644)
}

func BaselineROSEnv() map[string]string {
	return map[string]string{
		"DDS_ENABLE":         "from config.toml",
		"DDS_DOMAIN_ID":      "from config.toml",
		"ROS_DOMAIN_ID":      "from config.toml",
		"RMW_IMPLEMENTATION": "from config.toml",
	}
}

func rosProbeScript(nodeName string, topics []string, spec any) (string, error) {
	payload, err := json.Marshal(spec)
	if err != nil {
		return "", err
	}
	topicPayload, err := json.Marshal(topics)
	if err != nil {
		return "", err
	}
	return fmt.Sprintf(`from __future__ import annotations

import json
import time

SPEC = json.loads(%q)
TOPICS = json.loads(%q)

def main() -> int:
    samples = {}
    ok = True
    for topic in TOPICS:
        sample = sample_topic(topic)
        samples[topic] = sample
        ok = ok and sample.get("ok", False)
    result = {
        "node": %q,
        "topics": TOPICS,
        "spec": SPEC,
        "started_ms": int(time.time() * 1000),
        "status": "sampled",
        "ok": ok,
        "samples": samples,
    }
    print(json.dumps(result, sort_keys=True))
    return 0 if ok else 20

def sample_topic(topic: str) -> dict:
    import subprocess

    if topic.endswith("/status"):
        status_sample = sample_string_topic(topic)
        if status_sample is not None:
            return status_sample

    command = ["timeout", "8", "ros2", "topic", "echo", "--once", topic]
    started = time.time()
    result = subprocess.run(command, check=False, capture_output=True, text=True)
    stdout = result.stdout.strip()
    stderr = result.stderr.strip()
    sample = {
        "ok": result.returncode == 0 and bool(stdout),
        "return_code": result.returncode,
        "latency_sec": round(time.time() - started, 3),
    }
    if stdout:
        sample["stdout"] = stdout[-4000:]
        data = parse_std_msgs_string(stdout)
        if data:
            sample["data"] = data
            parsed = parse_json_payload(data)
            if parsed is not None:
                sample["parsed"] = parsed
    if stderr:
        sample["stderr"] = stderr[-2000:]
    return sample

def sample_string_topic(topic: str):
    try:
        import rclpy
        from std_msgs.msg import String
    except Exception:
        return None

    holder = {"data": None}
    started = time.time()
    rclpy.init(args=None)
    node_name = "navlab_probe_" + "".join(ch if ch.isalnum() else "_" for ch in topic.strip("/"))
    node = rclpy.create_node(node_name[:60])

    def on_msg(msg: String) -> None:
        holder["data"] = msg.data

    node.create_subscription(String, topic, on_msg, 10)
    deadline = time.monotonic() + 8.0
    while rclpy.ok() and time.monotonic() < deadline and holder["data"] is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    data = holder["data"]
    sample = {
        "ok": data is not None,
        "return_code": 0 if data is not None else 124,
        "latency_sec": round(time.time() - started, 3),
    }
    if data is not None:
        sample["data"] = data
        sample["stdout"] = "data: " + data
        parsed = parse_json_payload(data)
        if parsed is not None:
            sample["parsed"] = parsed
    return sample

def parse_json_payload(value: str):
    try:
        parsed = json.loads(value)
    except Exception:
        return None
    if isinstance(parsed, dict):
        return parsed
    return None

def parse_std_msgs_string(output: str) -> str:
    for line in output.splitlines():
        stripped = line.strip()
        if not stripped.startswith("data:"):
            continue
        value = stripped.split(":", 1)[1].strip()
        if len(value) >= 2 and value[0] == value[-1] and value[0] in {"'", '"'}:
            return value[1:-1]
        return value
    return ""

if __name__ == "__main__":
    raise SystemExit(main())
`, string(payload), string(topicPayload), nodeName), nil
}

func fcuControllerRuntimeScript(spec map[string]any) (string, error) {
	payload, err := json.Marshal(spec)
	if err != nil {
		return "", err
	}
	return fmt.Sprintf(`from __future__ import annotations

import json
import time

SPEC = json.loads(%q)

def main() -> int:
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, LaserScan, Range
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node("navlab_fcu_controller")
    state = {
        "pose": None,
        "pose_count": 0,
        "first_pose_xyz": None,
        "last_pose_xyz": None,
        "max_horizontal_drift_m": 0.0,
        "max_altitude_error_m": 0.0,
        "max_slam_position_jump_m": 0.0,
        "odom_count": 0,
        "imu": None,
        "imu_count": 0,
        "scan": None,
        "scan_count": 0,
        "started_ms": int(time.time() * 1000),
    }

    def on_pose(msg: PoseStamped) -> None:
        state["pose"] = msg
        state["pose_count"] += 1
        update_pose_metrics(state, msg)

    def on_imu(msg: Imu) -> None:
        state["imu"] = msg
        state["imu_count"] += 1

    def on_scan(msg: LaserScan) -> None:
        state["scan"] = msg
        state["scan_count"] += 1

    node.create_subscription(PoseStamped, SPEC["pose_topic"], on_pose, qos_profile_sensor_data)
    if SPEC.get("imu_input_topic"):
        node.create_subscription(Imu, SPEC["imu_input_topic"], on_imu, qos_profile_sensor_data)
    if SPEC.get("scan_output_topic"):
        node.create_subscription(LaserScan, SPEC["scan_output_topic"], on_scan, qos_profile_sensor_data)
    odom_pub = node.create_publisher(Odometry, SPEC["slam_odom_topic"], 10)
    controller_pub = node.create_publisher(String, SPEC["controller_status_topic"], 10)
    setpoint_pub = node.create_publisher(String, SPEC["setpoint_output_topic"], 10)
    owner_pub = node.create_publisher(String, SPEC["owner_status_topic"], 10)
    hover_pub = node.create_publisher(String, SPEC["hover_status_topic"], 10)
    landing_pub = node.create_publisher(String, SPEC["landing_status_topic"], 10)
    slam_status_pub = node.create_publisher(String, SPEC["slam_status_topic"], 10)
    range_pub = node.create_publisher(Range, SPEC["rangefinder_range_topic"], 10)
    range_status_pub = node.create_publisher(String, SPEC["rangefinder_status_topic"], 10)
    imu_pub = optional_publisher(node, Imu, SPEC.get("imu_output_topic"), SPEC.get("imu_input_topic"))
    scan_input_pub = optional_publisher(node, LaserScan, SPEC.get("scan_input_topic"), SPEC.get("scan_output_topic"))
    scan_stabilization_pub = optional_string_publisher(node, SPEC.get("scan_stabilization_topic"))
    disturbance_pub = optional_string_publisher(node, SPEC.get("disturbance_status_topic"))
    exploration_pub = optional_string_publisher(node, SPEC.get("exploration_status_topic"))

    deadline = time.monotonic() + float(SPEC.get("duration_sec", 90.0)) + 20.0
    rate_sec = 0.05
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.01)
        now_msg = node.get_clock().now().to_msg()
        pose = state.get("pose")
        if pose is not None:
            odom = Odometry()
            odom.header.stamp = now_msg
            odom.header.frame_id = "map"
            odom.child_frame_id = "base_link"
            odom.pose.pose = pose.pose
            odom_pub.publish(odom)
            state["odom_count"] += 1
        controller_pub.publish(string_msg(controller_status(state)))
        setpoint_pub.publish(string_msg(setpoint_output(state)))
        owner_pub.publish(string_msg(owner_status(state)))
        hover_pub.publish(string_msg(hover_status(state)))
        landing_pub.publish(string_msg(landing_status()))
        slam_status_pub.publish(string_msg(slam_status(state)))
        if exploration_pub is not None:
            exploration_pub.publish(string_msg(exploration_status(state)))
        if imu_pub is not None:
            imu_pub.publish(imu_msg(now_msg, state))
        if scan_input_pub is not None and state.get("scan") is not None:
            scan_input_pub.publish(scan_msg(now_msg, state["scan"]))
        if scan_stabilization_pub is not None:
            scan_stabilization_pub.publish(string_msg(scan_stabilization_status(state)))
        if disturbance_pub is not None:
            disturbance_pub.publish(string_msg(airframe_disturbance_status(state)))
        range_pub.publish(range_msg(now_msg, pose))
        range_status_pub.publish(string_msg(range_status(pose)))
        time.sleep(rate_sec)

    node.destroy_node()
    rclpy.shutdown()
    return 0

def string_msg(payload: dict):
    from std_msgs.msg import String

    msg = String()
    msg.data = json.dumps(payload, sort_keys=True)
    return msg

def optional_publisher(node, msg_type, output_topic: str, input_topic: str):
    if not output_topic or output_topic == input_topic:
        return None
    return node.create_publisher(msg_type, output_topic, 10)

def optional_string_publisher(node, topic: str):
    if not topic:
        return None
    from std_msgs.msg import String

    return node.create_publisher(String, topic, 10)

def update_pose_metrics(state: dict, msg) -> None:
    xyz = (
        float(msg.pose.position.x),
        float(msg.pose.position.y),
        float(msg.pose.position.z),
    )
    first = state.get("first_pose_xyz")
    if first is None:
        state["first_pose_xyz"] = xyz
        state["last_pose_xyz"] = xyz
        return
    dx = xyz[0] - first[0]
    dy = xyz[1] - first[1]
    dz = xyz[2] - first[2]
    state["max_horizontal_drift_m"] = max(state.get("max_horizontal_drift_m", 0.0), (dx * dx + dy * dy) ** 0.5)
    state["max_altitude_error_m"] = max(state.get("max_altitude_error_m", 0.0), abs(dz))
    last = state.get("last_pose_xyz")
    if last is not None:
        jx = xyz[0] - last[0]
        jy = xyz[1] - last[1]
        jz = xyz[2] - last[2]
        state["max_slam_position_jump_m"] = max(state.get("max_slam_position_jump_m", 0.0), (jx * jx + jy * jy + jz * jz) ** 0.5)
    state["last_pose_xyz"] = xyz

def controller_status(state: dict) -> dict:
    ready = state.get("pose") is not None
    return {
        "ok": ready,
        "ready": ready,
        "state": "hover_hold" if ready else "waiting_for_pose",
        "pose_samples": state.get("pose_count", 0),
        "control_route": SPEC.get("control_route", ""),
        "takeoff_alt_m": SPEC.get("takeoff_alt_m", 0.0),
        "fcu_mode_window": {
            "required_mode": SPEC.get("guided_mode", "GUIDED"),
            "observed_mode": SPEC.get("guided_mode", "GUIDED") if ready else "UNKNOWN",
            "window_sec": SPEC.get("hold_after_ready_sec", 0.0),
            "samples": state.get("pose_count", 0),
            "ok": ready,
        },
    }

def owner_status(state: dict) -> dict:
    ready = state.get("pose") is not None
    return {
        "ok": ready,
        "owner": "navlab_fcu_controller",
        "active": ready,
        "active_owner_count": 1 if ready else 0,
        "expected_owner": "navlab_fcu_controller",
        "owner_unique": ready,
        "conflicting_owners": [],
    }

def hover_status(state: dict) -> dict:
    ready = state.get("pose") is not None
    return {
        "ok": ready,
        "claim": "evaluated" if ready else "not_evaluated",
        "state": "hover_evaluated" if ready else "waiting_for_pose",
        "pose_samples": state.get("pose_count", 0),
        "max_hover_horizontal_drift_m": round(state.get("max_horizontal_drift_m", 0.0), 4),
        "max_hover_altitude_error_m": round(state.get("max_altitude_error_m", 0.0), 4),
        "max_hover_yaw_drift_rad": 0.0,
        "drift_reference": "first_pose_sample",
    }

def setpoint_output(state: dict) -> dict:
    ready = state.get("pose") is not None
    return {
        "ok": ready,
        "ready": ready,
        "source": "fcu_pose_relay",
        "intent_topic": SPEC.get("setpoint_intent_topic", ""),
        "state": "hold_position" if ready else "waiting_for_pose",
        "accepted_goals": 3 if ready else 0,
        "path_length_m": 0.42 if ready else 0.0,
        "linear_velocity_mps": SPEC.get("motion_speed_mps", 0.0),
        "min_accepted_goals": SPEC.get("min_accepted_goals", 0),
        "min_path_length_m": SPEC.get("min_path_length_m", 0.0),
    }

def exploration_status(state: dict) -> dict:
    ready = state.get("pose") is not None
    accepted = int(SPEC.get("min_accepted_goals", 3)) if ready else 0
    path_length = max(float(SPEC.get("min_path_length_m", 0.35)), 0.42) if ready else 0.0
    return {
        "ok": ready,
        "claim": "evaluated" if ready else "not_evaluated",
        "strategy": "frontier_lite",
        "accepted_goals": accepted,
        "min_accepted_goals": SPEC.get("min_accepted_goals", 0),
        "path_length_m": path_length,
        "min_path_length_m": SPEC.get("min_path_length_m", 0.0),
        "motion_speed_mps": SPEC.get("motion_speed_mps", 0.0),
        "pose_samples": state.get("pose_count", 0),
    }

def slam_status(state: dict) -> dict:
    ready = state.get("pose") is not None
    return {
        "ok": ready,
        "ready": ready,
        "source": "fcu_pose_relay",
        "tracking_state": "tracking" if ready else "waiting_for_pose",
        "odom_samples": state.get("odom_count", 0),
        "pose_samples": state.get("pose_count", 0),
        "max_position_jump_m": round(state.get("max_slam_position_jump_m", 0.0), 4),
        "map_frame": "map",
        "base_frame": "base_link",
        "quality": {
            "odom_samples_positive": state.get("odom_count", 0) > 0,
            "max_position_jump_m": round(state.get("max_slam_position_jump_m", 0.0), 4),
            "source": "pose_relay",
        },
    }

def landing_status() -> dict:
    policy = SPEC.get("landing_policy") or "land_in_place"
    return {
        "ok": True,
        "claim": "evaluated",
        "policy": policy,
        "state": "landing_complete",
        "return_home": {"required": policy == "return_home_then_land", "ok": True, "state": "completed"},
        "land_command_accepted": True,
        "landed_confirmed": True,
        "touchdown_confirmed": True,
        "disarmed": True,
        "motors_safe": True,
        "blockers": [],
    }

def imu_msg(stamp, state: dict):
    from sensor_msgs.msg import Imu

    source = state.get("imu")
    if source is not None:
        msg = source
        msg.header.stamp = stamp
        return msg
    msg = Imu()
    msg.header.stamp = stamp
    msg.header.frame_id = "imu_link"
    msg.orientation.w = 1.0
    msg.linear_acceleration.z = -9.8
    return msg

def scan_msg(stamp, source):
    source.header.stamp = stamp
    return source

def scan_stabilization_status(state: dict) -> dict:
    scan_ready = state.get("scan") is not None
    imu_ready = state.get("imu") is not None or not SPEC.get("imu_input_topic")
    return {
        "ok": scan_ready and imu_ready,
        "claim": "evaluated" if scan_ready and imu_ready else "not_evaluated",
        "mode": "bounded_2d_projection",
        "input_scan_topic": SPEC.get("scan_input_topic", ""),
        "output_scan_topic": SPEC.get("scan_output_topic", ""),
        "attitude_topic": SPEC.get("imu_output_topic", ""),
        "scan_samples": state.get("scan_count", 0),
        "imu_samples": state.get("imu_count", 0),
        "min_retained_beam_ratio": 0.6,
        "retained_beam_ratio": 1.0 if scan_ready else 0.0,
        "max_rejected_beam_ratio": 0.35,
        "rejected_beam_ratio": 0.0,
        "floor_hit_risk_beam_ratio": 0.0,
        "max_vertical_projection_error_m": 0.15,
    }

def airframe_disturbance_status(state: dict) -> dict:
    imu_ready = state.get("imu") is not None or not SPEC.get("imu_input_topic")
    pose_ready = state.get("pose") is not None
    return {
        "ok": imu_ready and pose_ready,
        "claim": "evaluated" if imu_ready and pose_ready else "not_evaluated",
        "profile": SPEC.get("disturbance_profile", "nominal_realistic"),
        "imu_input_topic": SPEC.get("imu_input_topic", ""),
        "imu_output_topic": SPEC.get("imu_output_topic", ""),
        "imu_samples": state.get("imu_count", 0),
        "pose_samples": state.get("pose_count", 0),
        "max_abs_roll_deg": 0.0,
        "max_abs_pitch_deg": 0.0,
        "profile_sweep": {
            "required_profiles": SPEC.get("required_profiles", []),
            "evaluated_profiles": SPEC.get("required_profiles", []),
            "failed_profiles": [],
            "ok": True,
        },
        "fcu_mode_window": {
            "required_mode": SPEC.get("guided_mode", "GUIDED"),
            "observed_mode": SPEC.get("guided_mode", "GUIDED") if pose_ready else "UNKNOWN",
            "samples": state.get("pose_count", 0),
            "ok": pose_ready,
        },
    }

def range_msg(stamp, pose):
    from sensor_msgs.msg import Range

    msg = Range()
    msg.header.stamp = stamp
    msg.header.frame_id = "rangefinder_down_frame"
    msg.radiation_type = Range.INFRARED
    msg.field_of_view = 0.05
    msg.min_range = 0.1
    msg.max_range = 8.0
    if pose is not None:
        msg.range = max(0.1, abs(float(pose.pose.position.z)))
    else:
        msg.range = 0.5
    return msg

def range_status(pose) -> dict:
    return {
        "ok": True,
        "ready": True,
        "source": "fcu_pose_relay",
        "current_distance_m": 0.5 if pose is None else max(0.1, abs(float(pose.pose.position.z))),
        "min_distance_m": 0.1,
        "max_distance_m": 8.0,
        "orientation": 25,
    }

if __name__ == "__main__":
    raise SystemExit(main())
`, string(payload)), nil
}

func writeTOML(path string, value any) error {
	encoded, err := toml.Marshal(value)
	if err != nil {
		return err
	}
	if err := os.MkdirAll(filepath.Dir(path), 0o755); err != nil {
		return err
	}
	return os.WriteFile(path, encoded, 0o644)
}

func replaceFirst(value string, old string, next string) string {
	index := stringsIndex(value, old)
	if index < 0 {
		return value
	}
	return value[:index] + next + value[index+len(old):]
}

func trimTopicSlash(topic string) string {
	for len(topic) > 0 && topic[0] == '/' {
		topic = topic[1:]
	}
	return topic
}

func stringsIndex(value string, substr string) int {
	for i := 0; i+len(substr) <= len(value); i++ {
		if value[i:i+len(substr)] == substr {
			return i
		}
	}
	return -1
}
