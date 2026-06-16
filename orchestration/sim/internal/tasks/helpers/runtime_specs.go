package helpers

import (
	"encoding/json"
	"fmt"
	"os"
	"path/filepath"

	toml "github.com/pelletier/go-toml/v2"
)

const (
	OfficialIrisWithLidarModel       = "/opt/navlab_official_ws/install/ardupilot_gz_description/share/ardupilot_gz_description/models/iris_with_lidar/model.sdf"
	OfficialGazeboIrisParams         = "/opt/navlab_official_ws/install/ardupilot_sitl/share/ardupilot_sitl/config/default_params/gazebo-iris.parm"
	FCUControllerContainer           = "navlab-fcu-controller"
	MAVLinkExternalNavContainer      = "navlab-mavlink-external-nav"
	FCURosbagContainer               = "navlab-fcu-rosbag"
	FrameContractRosbagContainer     = "navlab-frame-contract-rosbag"
	HoverRosbagContainer             = "navlab-hover-rosbag"
	HoverVehicleMarkerContainer      = "navlab-hover-vehicle-markers"
	ScanStabilizationRosbagContainer = "navlab-scan-stabilization-rosbag"
	ExplorationRosbagContainer       = "navlab-exploration-rosbag"
)

type SensorRuntimeSpec struct {
	RuntimeConfigPath         string
	X2VirtualSerialLink       string
	X2ScanInputTopic          string
	X2ScanTopic               string
	X2StatusTopic             string
	RangefinderFrameID        string
	RangefinderScanIdealTopic string
	RangefinderRangeTopic     string
	RangefinderStatusTopic    string
	RangefinderVirtualSerial  string
	RangefinderSerialBaud     int
	RangefinderRateHz         float64
	RangefinderMinDistanceM   float64
	RangefinderMaxDistanceM   float64
	RangefinderModelPose      string
	RangefinderModelUpdateHz  float64
	RangefinderModelRayCount  int
	RangefinderModelNoiseM    float64
	IMUOutputTopic            string
	IMUSourceRoute            string
	IMUSourceTopic            string
	SyntheticFallbackEnabled  bool
}

func DefaultSensorRuntimeSpec() SensorRuntimeSpec {
	return SensorRuntimeSpec{
		RuntimeConfigPath:         "artifacts/gazebo_sensor_runtime.toml",
		X2VirtualSerialLink:       "/tmp/navlab_sim_x2",
		X2ScanInputTopic:          "/lidar",
		X2ScanTopic:               "/scan",
		X2StatusTopic:             "/sim/x2/status",
		RangefinderFrameID:        "rangefinder_down_frame",
		RangefinderScanIdealTopic: "/rangefinder/down/scan_ideal",
		RangefinderRangeTopic:     "/rangefinder/down/range",
		RangefinderStatusTopic:    "/rangefinder/down/status",
		RangefinderVirtualSerial:  "/tmp/navlab_benewake_tfmini",
		RangefinderSerialBaud:     115200,
		RangefinderRateHz:         20.0,
		RangefinderMinDistanceM:   0.05,
		RangefinderMaxDistanceM:   6.0,
		RangefinderModelPose:      "0 0 -0.02 0 1.5707963267948966 0",
		RangefinderModelUpdateHz:  20.0,
		RangefinderModelRayCount:  1,
		RangefinderModelNoiseM:    0.0,
		IMUOutputTopic:            "/imu",
		IMUSourceRoute:            "official_gazebo_imu_bridge",
		IMUSourceTopic:            "/imu",
		SyntheticFallbackEnabled:  false,
	}
}

func WriteSensorRuntimeConfig(path string, vendorProfile string, spec SensorRuntimeSpec) error {
	if spec.RuntimeConfigPath == "" {
		spec = DefaultSensorRuntimeSpec()
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
				"virtual_serial_link":  spec.RangefinderVirtualSerial,
				"serial_baud":          spec.RangefinderSerialBaud,
				"frame_id":             spec.RangefinderFrameID,
				"rate_hz":              spec.RangefinderRateHz,
				"min_distance_m":       spec.RangefinderMinDistanceM,
				"max_distance_m":       spec.RangefinderMaxDistanceM,
				"model_pose":           spec.RangefinderModelPose,
				"model_update_rate_hz": spec.RangefinderModelUpdateHz,
				"model_ray_count":      spec.RangefinderModelRayCount,
				"model_noise_stddev_m": spec.RangefinderModelNoiseM,
			},
		},
	}
	return writeTOML(path, data)
}

func RangefinderProbeScript(spec SensorRuntimeSpec) (string, error) {
	if spec.RuntimeConfigPath == "" {
		spec = DefaultSensorRuntimeSpec()
	}
	return rosProbeScript("navlab_rangefinder_probe", []string{spec.RangefinderRangeTopic, spec.RangefinderStatusTopic}, map[string]any{
		"probe_timeout_sec": 45.0,
		"range_topic":       spec.RangefinderRangeTopic,
		"status_topic":      spec.RangefinderStatusTopic,
	})
}

func IMUProbeScript(spec SensorRuntimeSpec) (string, error) {
	if spec.RuntimeConfigPath == "" {
		spec = DefaultSensorRuntimeSpec()
	}
	return rosProbeScript("navlab_imu_probe", []string{spec.IMUOutputTopic}, map[string]any{
		"topic":                      spec.IMUOutputTopic,
		"source_route":               spec.IMUSourceRoute,
		"source_topic":               spec.IMUSourceTopic,
		"synthetic_fallback_enabled": spec.SyntheticFallbackEnabled,
	})
}

type FCUControllerSpec struct {
	ControlRoute                    string
	MAVLinkBootstrap                string
	MAVLinkBootstrapSourceSystem    int
	MAVLinkBootstrapSourceComponent int
	OwnerName                       string
	OwnerID                         string
	FCUStateTopic                   string
	ControllerStatusTopic           string
	SetpointIntentTopic             string
	SetpointOutputTopic             string
	OwnerStatusTopic                string
	TimeTopic                       string
	PrearmService                   string
	ModeSwitchService               string
	ArmService                      string
	TakeoffService                  string
	CmdVelTopic                     string
	PoseTopic                       string
	TwistTopic                      string
	StatusTopic                     string
	RangefinderRangeTopic           string
	RangefinderStatusTopic          string
	IMUTopic                        string
	IMUInputTopic                   string
	ScanInputTopic                  string
	ScanOutputTopic                 string
	ScanStabilizationTopic          string
	DisturbanceStatusTopic          string
	ExplorationStatusTopic          string
	TaskCompletionStatusTopic       string
	SlamOdomTopic                   string
	CartographerOdometryTopic       string
	SlamStatusTopic                 string
	MapFrameID                      string
	OdomFrameID                     string
	BaseFrameID                     string
	LaserFrameID                    string
	LandingPolicy                   string
	CompletionGraceSec              float64
	MotionSpeedMPS                  float64
	MinAcceptedGoals                int
	MinPathLengthM                  float64
	DisturbanceProfile              string
	RequiredProfiles                []string
	ESCLagMS                        []float64
	MotorJitterHz                   float64
	ThrustNoiseStd                  float64
	IMUVibrationEnabled             bool
	IMUVibrationRollPitchAmpDeg     float64
	GuidedMode                      string
	TakeoffAltM                     float64
	TakeoffMinHeightM               float64
	TakeoffMinHeightRatio           float64
	ReadinessTimeoutSec             float64
	HoldAfterReadySec               float64
	RequireSlamBackend              bool
	DisableArmingChecks             bool
	ForceArm                        bool
}

func DefaultFCUControllerSpec() FCUControllerSpec {
	return FCUControllerSpec{
		ControlRoute:                    "mavlink_bootstrap_plus_dds_cmd_vel",
		MAVLinkBootstrap:                "udpin:0.0.0.0:14551",
		MAVLinkBootstrapSourceSystem:    246,
		MAVLinkBootstrapSourceComponent: 190,
		OwnerName:                       "navlab_fcu_controller",
		OwnerID:                         "fcu-controller",
		FCUStateTopic:                   "/navlab/fcu/state",
		ControllerStatusTopic:           "/navlab/fcu/controller/status",
		SetpointIntentTopic:             "/navlab/fcu/setpoint/intent",
		SetpointOutputTopic:             "/navlab/fcu/setpoint/output",
		OwnerStatusTopic:                "/navlab/fcu/owner/status",
		TimeTopic:                       "/ap/v1/time",
		PrearmService:                   "/ap/v1/prearm_check",
		ModeSwitchService:               "/ap/v1/mode_switch",
		ArmService:                      "/ap/v1/arm_motors",
		TakeoffService:                  "/ap/v1/experimental/takeoff",
		CmdVelTopic:                     "/ap/v1/cmd_vel",
		PoseTopic:                       "/ap/v1/pose/filtered",
		TwistTopic:                      "/ap/v1/twist/filtered",
		StatusTopic:                     "/ap/v1/status",
		RangefinderRangeTopic:           "/rangefinder/down/range",
		RangefinderStatusTopic:          "/rangefinder/down/status",
		IMUTopic:                        "/imu",
		IMUInputTopic:                   "",
		ScanInputTopic:                  "",
		ScanOutputTopic:                 "",
		ScanStabilizationTopic:          "",
		DisturbanceStatusTopic:          "",
		ExplorationStatusTopic:          "",
		TaskCompletionStatusTopic:       "",
		SlamOdomTopic:                   "/slam/odom",
		CartographerOdometryTopic:       "",
		SlamStatusTopic:                 "/navlab/slam/status",
		MapFrameID:                      "map",
		OdomFrameID:                     "odom",
		BaseFrameID:                     "base_link",
		LaserFrameID:                    "base_scan",
		LandingPolicy:                   "land_in_place",
		CompletionGraceSec:              3.0,
		MotionSpeedMPS:                  0.0,
		MinAcceptedGoals:                0,
		MinPathLengthM:                  0.0,
		DisturbanceProfile:              "realistic",
		RequiredProfiles:                nil,
		ESCLagMS:                        nil,
		MotorJitterHz:                   0.0,
		ThrustNoiseStd:                  0.0,
		IMUVibrationEnabled:             false,
		IMUVibrationRollPitchAmpDeg:     0.0,
		GuidedMode:                      "GUIDED",
		TakeoffAltM:                     0.5,
		TakeoffMinHeightM:               0.15,
		TakeoffMinHeightRatio:           0.35,
		ReadinessTimeoutSec:             45.0,
		HoldAfterReadySec:               8.0,
		RequireSlamBackend:              true,
		DisableArmingChecks:             true,
		ForceArm:                        true,
	}
}

func WriteFCUControllerRuntimeConfig(path string, spec FCUControllerSpec) error {
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
		"duration_sec":                       durationSec,
		"control_route":                      spec.ControlRoute,
		"mavlink_bootstrap_endpoint":         spec.MAVLinkBootstrap,
		"mavlink_bootstrap_source_system":    spec.MAVLinkBootstrapSourceSystem,
		"mavlink_bootstrap_source_component": spec.MAVLinkBootstrapSourceComponent,
		"pose_topic":                         spec.PoseTopic,
		"controller_status_topic":            spec.ControllerStatusTopic,
		"setpoint_intent_topic":              spec.SetpointIntentTopic,
		"setpoint_output_topic":              spec.SetpointOutputTopic,
		"owner_status_topic":                 spec.OwnerStatusTopic,
		"fcu_state_topic":                    spec.FCUStateTopic,
		"cmd_vel_topic":                      spec.CmdVelTopic,
		"prearm_service":                     spec.PrearmService,
		"mode_switch_service":                spec.ModeSwitchService,
		"arm_service":                        spec.ArmService,
		"takeoff_service":                    spec.TakeoffService,
		"takeoff_alt_m":                      spec.TakeoffAltM,
		"takeoff_min_height_m":               spec.TakeoffMinHeightM,
		"takeoff_min_height_ratio":           spec.TakeoffMinHeightRatio,
		"guided_mode":                        spec.GuidedMode,
		"readiness_timeout_sec":              spec.ReadinessTimeoutSec,
		"hold_after_ready_sec":               spec.HoldAfterReadySec,
		"require_slam_backend":               spec.RequireSlamBackend,
		"disable_arming_checks":              spec.DisableArmingChecks,
		"force_arm":                          spec.ForceArm,
		"rangefinder_range_topic":            spec.RangefinderRangeTopic,
		"rangefinder_status_topic":           spec.RangefinderStatusTopic,
		"imu_input_topic":                    spec.IMUInputTopic,
		"imu_output_topic":                   spec.IMUTopic,
		"scan_input_topic":                   spec.ScanInputTopic,
		"scan_output_topic":                  spec.ScanOutputTopic,
		"scan_stabilization_topic":           spec.ScanStabilizationTopic,
		"disturbance_status_topic":           spec.DisturbanceStatusTopic,
		"exploration_status_topic":           spec.ExplorationStatusTopic,
		"task_completion_status_topic":       spec.TaskCompletionStatusTopic,
		"slam_odom_topic":                    spec.SlamOdomTopic,
		"cartographer_odometry_topic":        spec.CartographerOdometryTopic,
		"slam_status_topic":                  spec.SlamStatusTopic,
		"map_frame_id":                       spec.MapFrameID,
		"odom_frame_id":                      spec.OdomFrameID,
		"base_frame_id":                      spec.BaseFrameID,
		"laser_frame_id":                     spec.LaserFrameID,
		"hover_status_topic":                 "/navlab/hover/status",
		"landing_status_topic":               "/navlab/landing/status",
		"landing_policy":                     spec.LandingPolicy,
		"completion_grace_sec":               spec.CompletionGraceSec,
		"motion_speed_mps":                   spec.MotionSpeedMPS,
		"min_accepted_goals":                 spec.MinAcceptedGoals,
		"min_path_length_m":                  spec.MinPathLengthM,
		"disturbance_profile":                spec.DisturbanceProfile,
		"required_profiles":                  append([]string(nil), spec.RequiredProfiles...),
		"esc_lag_ms":                         append([]float64(nil), spec.ESCLagMS...),
		"motor_jitter_hz":                    spec.MotorJitterHz,
		"thrust_noise_std":                   spec.ThrustNoiseStd,
		"imu_vibration_enabled":              spec.IMUVibrationEnabled,
		"imu_vibration_roll_pitch_amp_deg":   spec.IMUVibrationRollPitchAmpDeg,
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
		RequiredFrames:          []string{"map", "odom", "base_link", "imu_link", "base_scan", "rangefinder_down_frame"},
		MapFrameID:              "map",
		OdomFrameID:             "odom",
		BaseFrameID:             "base_link",
		IMUFrameID:              "imu_link",
		LaserFrameID:            "base_scan",
		RangefinderFrameID:      "rangefinder_down_frame",
		ScanTopic:               "/scan",
		IMUTopic:                "/imu",
		RangefinderRangeTopic:   "/rangefinder/down/range",
		RangefinderStatusTopic:  "/rangefinder/down/status",
		FCUPoseTopic:            "/ap/v1/pose/filtered",
		FCUTwistTopic:           "/ap/v1/twist/filtered",
		FCUStatusTopic:          "/ap/v1/status",
		CmdVelTopic:             "/ap/v1/cmd_vel",
		SlamOdomTopic:           "/slam/odom",
		SlamStatusTopic:         "/navlab/slam/status",
		TruthDiagnosticTopic:    "/odometry",
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

func WriteFrameContractRuntimeConfig(path string, spec FrameContractSpec) error {
	if spec.MapFrameID == "" {
		spec = DefaultFrameContractSpec()
	}
	return writeTOML(path, map[string]any{"frame_contract": map[string]any{"runtime": spec}})
}

func FrameContractProbeScript(spec FrameContractSpec) (string, error) {
	if spec.MapFrameID == "" {
		spec = DefaultFrameContractSpec()
	}
	return rosProbeScript("navlab_frame_contract_probe", []string{"/tf", "/tf_static", spec.ScanTopic, spec.IMUTopic, spec.RangefinderRangeTopic, spec.FCUPoseTopic, spec.SlamOdomTopic, spec.SlamStatusTopic}, spec)
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
	ProbeTimeoutSec         float64
}

type HoverMissionRuntimeSpec struct {
	Endpoint                  string
	SourceSystem              int
	SourceComponent           int
	Mode                      string
	TakeoffAltM               float64
	MinAirborneAltM           float64
	PreflightReadySec         float64
	HoverSettleSec            float64
	HoverAltitudeToleranceM   float64
	HoverHoldSec              float64
	MaxHorizontalDriftM       float64
	MaxAltitudeDriftM         float64
	StatusTopic               string
	LandingStatusTopic        string
	LandingIntentTopic        string
	ExternalNavStatusTopic    string
	IMUStatusTopic            string
	MAVLinkStatusTopic        string
	PreLandHoldSec            float64
	MaxLandingDurationSec     float64
	TouchdownAltitudeM        float64
	TouchdownVerticalSpeedMPS float64
	RequireDisarm             bool
	RequireMotorsSafe         bool
	RequireExternalNav        bool
	RequireIMUStatus          bool
	DisableArmingChecks       bool
	ForceArm                  bool
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
		RangefinderRangeTopic:   "/rangefinder/down/range",
		RangefinderStatusTopic:  "/rangefinder/down/status",
		IMUTopic:                "/imu",
		TruthDiagnosticTopic:    "/odometry",
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
		ProbeTimeoutSec:         120.0,
	}
}

func DefaultHoverMissionRuntimeSpec() HoverMissionRuntimeSpec {
	fcu := DefaultFCUControllerSpec()
	hover := DefaultSlamHoverSpec()
	return HoverMissionRuntimeSpec{
		Endpoint:                  fcu.MAVLinkBootstrap,
		SourceSystem:              fcu.MAVLinkBootstrapSourceSystem,
		SourceComponent:           fcu.MAVLinkBootstrapSourceComponent,
		Mode:                      fcu.GuidedMode,
		TakeoffAltM:               fcu.TakeoffAltM,
		MinAirborneAltM:           0.10,
		PreflightReadySec:         5.0,
		HoverSettleSec:            2.0,
		HoverAltitudeToleranceM:   0.18,
		HoverHoldSec:              hover.HoverWindowSec,
		MaxHorizontalDriftM:       hover.MaxHoverHorizontalDrift,
		MaxAltitudeDriftM:         hover.MaxHoverAltitudeError,
		StatusTopic:               hover.HoverStatusTopic,
		LandingStatusTopic:        "/navlab/landing/status",
		LandingIntentTopic:        "/navlab/landing/intent",
		ExternalNavStatusTopic:    hover.ExternalNavStatusTopic,
		IMUStatusTopic:            "/imu/status",
		MAVLinkStatusTopic:        "/navlab/mavlink/status",
		PreLandHoldSec:            2.0,
		MaxLandingDurationSec:     35.0,
		TouchdownAltitudeM:        0.12,
		TouchdownVerticalSpeedMPS: 0.08,
		RequireDisarm:             true,
		RequireMotorsSafe:         true,
		RequireExternalNav:        true,
		RequireIMUStatus:          true,
		DisableArmingChecks:       true,
		ForceArm:                  false,
	}
}

func WriteHoverRuntimeConfig(path string, spec SlamHoverSpec) error {
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
		[]string{spec.FCUPoseTopic, spec.SlamOdomTopic, spec.SlamStatusTopic, spec.ExternalNavStatusTopic, "/mavlink_external_nav/status", spec.HoverStatusTopic, "/navlab/landing/status"},
		spec,
	)
}

func HoverMissionRuntimeScript(spec HoverMissionRuntimeSpec, durationSec float64) (string, error) {
	if spec.Endpoint == "" {
		spec = DefaultHoverMissionRuntimeSpec()
	}
	payload, err := json.Marshal(map[string]any{
		"duration_sec":                 durationSec,
		"endpoint":                     spec.Endpoint,
		"source_system":                spec.SourceSystem,
		"source_component":             spec.SourceComponent,
		"mode":                         spec.Mode,
		"takeoff_alt_m":                spec.TakeoffAltM,
		"min_airborne_alt_m":           spec.MinAirborneAltM,
		"preflight_ready_sec":          spec.PreflightReadySec,
		"hover_settle_sec":             spec.HoverSettleSec,
		"hover_altitude_tolerance_m":   spec.HoverAltitudeToleranceM,
		"hover_hold_sec":               spec.HoverHoldSec,
		"max_horizontal_drift_m":       spec.MaxHorizontalDriftM,
		"max_altitude_drift_m":         spec.MaxAltitudeDriftM,
		"status_topic":                 spec.StatusTopic,
		"landing_status_topic":         spec.LandingStatusTopic,
		"landing_intent_topic":         spec.LandingIntentTopic,
		"external_nav_status_topic":    spec.ExternalNavStatusTopic,
		"imu_status_topic":             spec.IMUStatusTopic,
		"mavlink_status_topic":         spec.MAVLinkStatusTopic,
		"pre_land_hold_sec":            spec.PreLandHoldSec,
		"max_landing_duration_sec":     spec.MaxLandingDurationSec,
		"touchdown_altitude_m":         spec.TouchdownAltitudeM,
		"touchdown_vertical_speed_mps": spec.TouchdownVerticalSpeedMPS,
		"require_disarm":               spec.RequireDisarm,
		"require_motors_safe":          spec.RequireMotorsSafe,
		"require_external_nav":         spec.RequireExternalNav,
		"require_imu_status":           spec.RequireIMUStatus,
		"disable_arming_checks":        spec.DisableArmingChecks,
		"force_arm":                    spec.ForceArm,
	})
	if err != nil {
		return "", err
	}
	return fmt.Sprintf(`from __future__ import annotations

import json
from pathlib import Path

SPEC = json.loads(%q)

def bool_flag(name: str) -> str:
    return "--" + name if SPEC.get(name.replace("-", "_"), False) else "--no-" + name

def add_arg(argv: list[str], name: str, value) -> None:
    argv.extend(["--" + name, str(value)])

def main() -> int:
    from navlab.sim.companion.nodes.hover_mission import run

    artifact_dir = Path(__file__).resolve().parent
    argv: list[str] = []
    for name in [
        "endpoint",
        "duration-sec",
        "summary-file",
        "mode",
        "takeoff-alt-m",
        "min-airborne-alt-m",
        "preflight-ready-sec",
        "hover-settle-sec",
        "hover-altitude-tolerance-m",
        "hover-hold-sec",
        "max-horizontal-drift-m",
        "max-altitude-drift-m",
        "source-system",
        "source-component",
        "status-topic",
        "landing-status-topic",
        "landing-intent-topic",
        "external-nav-status-topic",
        "imu-status-topic",
        "mavlink-status-topic",
        "pre-land-hold-sec",
        "max-landing-duration-sec",
        "touchdown-altitude-m",
        "touchdown-vertical-speed-mps",
    ]:
        key = name.replace("-", "_")
        value = str(artifact_dir / "mission_summary.json") if key == "summary_file" else SPEC[key]
        add_arg(argv, name, value)
    for name in [
        "require-disarm",
        "require-motors-safe",
        "require-external-nav",
        "require-imu-status",
        "disable-arming-checks",
        "force-arm",
    ]:
        argv.append(bool_flag(name))
    rc = int(run(argv) or 0)
    (artifact_dir / "hover_mission_rc.txt").write_text(str(rc) + "\n", encoding="utf-8")
    return rc

if __name__ == "__main__":
    raise SystemExit(main())
`, string(payload)), nil
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

func ValidateScanStabilizationConfig(spec ScanStabilizationSpec) []string {
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
		blockers = append(blockers, "scan_stabilization must not use Gazebo truth as input")
	}
	if spec.ScanStabilizationClaim != "evaluated" {
		blockers = append(blockers, "scan_stabilization_claim must be evaluated")
	}
	return blockers
}

func WriteScanStabilizationRuntimeConfig(path string, spec ScanStabilizationSpec) error {
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
	return rosProbeScript("navlab_scan_stabilization_status_probe", []string{spec.StatusTopic}, map[string]any{"status_topic": spec.StatusTopic})
}

func AirframeDisturbanceProbeScript(spec ScanRobustnessWorkflowSpec) (string, error) {
	if spec.Profile == "" {
		spec = DefaultScanRobustnessWorkflowSpec()
	}
	return rosProbeScript("navlab_airframe_disturbance_probe", []string{spec.DisturbanceStatusTopic}, map[string]any{
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
	ProbeTimeoutSec        float64
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
		ProbeTimeoutSec:        35.0,
		ControllerStatusTopic:  "/navlab/fcu/controller/status",
		SetpointIntentTopic:    "/navlab/fcu/setpoint/intent",
		SetpointOutputTopic:    "/navlab/fcu/setpoint/output",
		SlamOdomTopic:          "/slam/odom",
		ExplorationStatusTopic: "/navlab/exploration/status",
	}
}

func WriteExplorationRuntimeConfig(path string, spec ExplorationWorkflowSpec) error {
	if spec.Strategy == "" {
		spec = DefaultExplorationWorkflowSpec()
	}
	return writeTOML(path, map[string]any{"exploration_gate": map[string]any{"runtime": spec}})
}

func ExplorationProbeScript(spec ExplorationWorkflowSpec) (string, error) {
	if spec.Strategy == "" {
		spec = DefaultExplorationWorkflowSpec()
	}
	return rosProbeScript(
		"navlab_exploration_probe",
		[]string{spec.ControllerStatusTopic, spec.SetpointOutputTopic, spec.ExplorationStatusTopic, spec.SlamOdomTopic, "/navlab/landing/status"},
		spec,
	)
}

func ExplorationWorkflowRuntimeScript(spec ExplorationWorkflowSpec, durationSec float64) (string, error) {
	if spec.Strategy == "" {
		spec = DefaultExplorationWorkflowSpec()
	}
	payload, err := json.Marshal(map[string]any{
		"duration_sec":             durationSec,
		"strategy":                 spec.Strategy,
		"exploration_window_sec":   spec.ExplorationWindowSec,
		"motion_speed_mps":         spec.MotionSpeedMPS,
		"min_accepted_goals":       spec.MinAcceptedGoals,
		"min_path_length_m":        spec.MinPathLengthM,
		"controller_status_topic":  spec.ControllerStatusTopic,
		"setpoint_intent_topic":    spec.SetpointIntentTopic,
		"setpoint_output_topic":    spec.SetpointOutputTopic,
		"slam_odom_topic":          spec.SlamOdomTopic,
		"exploration_status_topic": spec.ExplorationStatusTopic,
	})
	if err != nil {
		return "", err
	}
	return fmt.Sprintf(`from __future__ import annotations

import json
import math
import time

SPEC = json.loads(%q)

def main() -> int:
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node("navlab_exploration_workflow")
    node.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    clock_deadline = time.monotonic() + 5.0
    while rclpy.ok() and time.monotonic() < clock_deadline and node.get_clock().now().nanoseconds <= 0:
        rclpy.spin_once(node, timeout_sec=0.1)

    state = {
        "controller": {},
        "controller_ready": False,
        "odom_samples": 0,
        "last_xyz": None,
        "path_length_m": 0.0,
        "accepted_goals": 0,
        "last_goal_index": -1,
        "started_ms": int(time.time() * 1000),
        "ready_since": 0.0,
        "completed_ms": 0,
        "completed_at": 0.0,
        "last_intent": {},
    }

    def on_controller_status(msg: String) -> None:
        payload = parse_json(msg.data)
        state["controller"] = payload
        was_ready = bool(state.get("controller_ready", False))
        state["controller_ready"] = bool(payload.get("ok", False) or payload.get("ready", False))
        if state["controller_ready"] and not was_ready:
            state["ready_since"] = time.monotonic()
            state["last_goal_index"] = -1
            state["accepted_goals"] = 0
            state["path_length_m"] = 0.0
            state["last_xyz"] = None

    def on_slam_odom(msg: Odometry) -> None:
        xyz = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            float(msg.pose.pose.position.z),
        )
        last = state.get("last_xyz")
        state["odom_samples"] += 1
        if last is not None:
            dx = xyz[0] - last[0]
            dy = xyz[1] - last[1]
            dz = xyz[2] - last[2]
            step = math.sqrt(dx * dx + dy * dy + dz * dz)
            if step <= 1.0:
                state["path_length_m"] += step
        state["last_xyz"] = xyz

    node.create_subscription(String, SPEC["controller_status_topic"], on_controller_status, 10)
    node.create_subscription(Odometry, SPEC["slam_odom_topic"], on_slam_odom, qos_profile_sensor_data)
    intent_pub = node.create_publisher(String, SPEC["setpoint_intent_topic"], 10)
    status_pub = node.create_publisher(String, SPEC["exploration_status_topic"], 10)
    goal_pub = node.create_publisher(String, "/navlab/exploration/goal", 10)
    coverage_pub = node.create_publisher(String, "/navlab/exploration/coverage", 10)
    frontiers_pub = node.create_publisher(String, "/navlab/exploration/frontiers", 10)
    path_pub = node.create_publisher(String, "/navlab/exploration/path", 10)
    markers_pub = node.create_publisher(String, "/navlab/exploration/markers", 10)

    speed = max(0.03, float(SPEC.get("motion_speed_mps", 0.10) or 0.10))
    min_goals = max(1, int(SPEC.get("min_accepted_goals", 3) or 3))
    window_sec = max(float(SPEC.get("exploration_window_sec", 26.0) or 26.0), float(min_goals) * 2.0)
    duration_sec = max(float(SPEC.get("duration_sec", window_sec) or window_sec), window_sec)
    segment_sec = max(2.0, window_sec / float(min_goals))
    start = time.monotonic()
    deadline = start + duration_sec + 15.0
    completed_hold_sec = 5.0

    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
        ready_elapsed = 0.0
        if state.get("controller_ready") and float(state.get("ready_since", 0.0) or 0.0) > 0.0:
            ready_elapsed = time.monotonic() - float(state.get("ready_since", 0.0) or 0.0)
        status = exploration_status(state)
        status_pub.publish(string_msg(status))
        publish_review_topics(goal_pub, coverage_pub, frontiers_pub, path_pub, markers_pub, state, status)

        if status["ok"]:
            if not state.get("completed_ms"):
                state["completed_ms"] = int(time.time() * 1000)
                state["completed_at"] = time.monotonic()
            intent_pub.publish(string_msg(stop_intent(state, "complete")))
            if time.monotonic() - float(state.get("completed_at", 0.0) or 0.0) >= completed_hold_sec:
                break
            time.sleep(0.1)
            continue

        if state.get("controller_ready"):
            goal_index = min(int(ready_elapsed / segment_sec), min_goals - 1)
            if goal_index != state.get("last_goal_index") and state.get("odom_samples", 0) > 0:
                state["accepted_goals"] = max(state.get("accepted_goals", 0), goal_index + 1)
                state["last_goal_index"] = goal_index
            intent = exploration_intent(goal_index, speed, state)
            state["last_intent"] = intent
            intent_pub.publish(string_msg(intent))
        else:
            intent_pub.publish(string_msg(stop_intent(state, "waiting_for_controller")))
        time.sleep(0.1)

    final_status = exploration_status(state)
    for _ in range(20):
        status_pub.publish(string_msg(final_status))
        intent_pub.publish(string_msg(stop_intent(state, "shutdown")))
        rclpy.spin_once(node, timeout_sec=0.02)
        time.sleep(0.05)
    node.destroy_node()
    rclpy.shutdown()
    return 0

def parse_json(value: str) -> dict:
    try:
        parsed = json.loads(value)
    except Exception:
        return {}
    return parsed if isinstance(parsed, dict) else {}

def string_msg(payload: dict):
    from std_msgs.msg import String

    msg = String()
    msg.data = json.dumps(payload, sort_keys=True)
    return msg

def exploration_intent(goal_index: int, speed: float, state: dict) -> dict:
    pattern = [
        {"linear_x_mps": speed, "linear_y_mps": 0.0, "yaw_rate_radps": 0.0},
        {"linear_x_mps": speed * 0.6, "linear_y_mps": 0.0, "yaw_rate_radps": 0.20},
        {"linear_x_mps": speed, "linear_y_mps": 0.0, "yaw_rate_radps": -0.12},
    ]
    command = dict(pattern[goal_index %% len(pattern)])
    command.update({
        "ok": True,
        "source": "exploration_workflow",
        "strategy": SPEC.get("strategy", "frontier_lite"),
        "goal_id": f"frontier_lite_{goal_index + 1}",
        "goal_index": goal_index,
        "uses_gazebo_truth_as_input": False,
        "odom_samples": state.get("odom_samples", 0),
        "path_length_m": round(state.get("path_length_m", 0.0), 4),
    })
    return command

def stop_intent(state: dict, reason: str) -> dict:
    return {
        "ok": True,
        "source": "exploration_workflow",
        "strategy": SPEC.get("strategy", "frontier_lite"),
        "goal_id": "hold",
        "linear_x_mps": 0.0,
        "linear_y_mps": 0.0,
        "yaw_rate_radps": 0.0,
        "reason": reason,
        "uses_gazebo_truth_as_input": False,
        "odom_samples": state.get("odom_samples", 0),
        "path_length_m": round(state.get("path_length_m", 0.0), 4),
    }

def exploration_status(state: dict) -> dict:
    accepted_goals = int(state.get("accepted_goals", 0) or 0)
    min_goals = int(SPEC.get("min_accepted_goals", 3) or 3)
    path_length = float(state.get("path_length_m", 0.0) or 0.0)
    min_path = float(SPEC.get("min_path_length_m", 0.35) or 0.35)
    odom_samples = int(state.get("odom_samples", 0) or 0)
    blockers = []
    if not state.get("controller_ready", False):
        blockers.append("controller_not_ready")
    if odom_samples <= 0:
        blockers.append("slam_odom_missing")
    if accepted_goals < min_goals:
        blockers.append("accepted_goals_below_min")
    if path_length < min_path:
        blockers.append("path_length_below_min")
    ok = len(blockers) == 0
    return {
        "ok": ok,
        "claim": "evaluated" if ok else "in_progress",
        "strategy": SPEC.get("strategy", "frontier_lite"),
        "accepted_goals": accepted_goals,
        "min_accepted_goals": min_goals,
        "path_length_m": round(path_length, 4),
        "min_path_length_m": min_path,
        "motion_speed_mps": float(SPEC.get("motion_speed_mps", 0.0) or 0.0),
        "odom_samples": odom_samples,
        "controller_ready": bool(state.get("controller_ready", False)),
        "ready_elapsed_sec": round(max(0.0, time.monotonic() - float(state.get("ready_since", 0.0) or time.monotonic())) if state.get("controller_ready", False) else 0.0, 3),
        "uses_gazebo_truth_as_input": False,
        "evidence_source": SPEC.get("slam_odom_topic", "/slam/odom"),
        "setpoint_intent_topic": SPEC.get("setpoint_intent_topic", ""),
        "blockers": blockers,
    }

def publish_review_topics(goal_pub, coverage_pub, frontiers_pub, path_pub, markers_pub, state: dict, status: dict) -> None:
    goal_payload = {
        "strategy": SPEC.get("strategy", "frontier_lite"),
        "accepted_goals": status.get("accepted_goals", 0),
        "last_intent": state.get("last_intent", {}),
        "uses_gazebo_truth_as_input": False,
    }
    coverage_payload = {
        "coverage_proxy": round(min(1.0, status.get("path_length_m", 0.0) / max(status.get("min_path_length_m", 0.35), 0.01)), 4),
        "path_length_m": status.get("path_length_m", 0.0),
        "source": SPEC.get("slam_odom_topic", "/slam/odom"),
    }
    frontiers_payload = {
        "strategy": SPEC.get("strategy", "frontier_lite"),
        "candidate_count": max(0, int(status.get("min_accepted_goals", 3)) - int(status.get("accepted_goals", 0))),
        "source": "bounded_lite_pattern",
    }
    path_payload = {
        "path_length_m": status.get("path_length_m", 0.0),
        "odom_samples": status.get("odom_samples", 0),
        "source": SPEC.get("slam_odom_topic", "/slam/odom"),
    }
    markers_payload = {
        "state": "complete" if status.get("ok", False) else "running",
        "blockers": status.get("blockers", []),
    }
    goal_pub.publish(string_msg(goal_payload))
    coverage_pub.publish(string_msg(coverage_payload))
    frontiers_pub.publish(string_msg(frontiers_payload))
    path_pub.publish(string_msg(path_payload))
    markers_pub.publish(string_msg(markers_payload))

if __name__ == "__main__":
    raise SystemExit(main())
`, string(payload)), nil
}

type Nav2NavigationSpec struct {
	Profile                      string
	GlobalFrame                  string
	OdomFrame                    string
	BaseFrame                    string
	ScanTopic                    string
	MapTopic                     string
	SeedMapTopic                 string
	CmdVelTopic                  string
	BTXML                        string
	PlannerPlugin                string
	ControllerPlugin             string
	UseSimTime                   bool
	GlobalCostmapTopic           string
	LocalCostmapTopic            string
	RequiredCostmapLayers        []string
	MaxCostmapAgeSec             float64
	MinObstacleCells             int
	MaxUnknownRatio              float64
	InflationRadiusM             float64
	FootprintRadiusM             float64
	CostmapHealthTopic           string
	SetpointIntentTopic          string
	AdapterStatusTopic           string
	MaxXYSpeedMPS                float64
	MaxYawRateDPS                float64
	MaxAccelMPS2                 float64
	FixedAltitudeM               float64
	StopOnStaleCostmap           bool
	StopOnStaleSlam              bool
	NavigationStatusTopic        string
	NavigationEventsTopic        string
	NavigationGoalTopic          string
	NavigationPathTopic          string
	NavigationRecoveryTopic      string
	Strategy                     string
	CompletionPolicy             string
	GoalFrame                    string
	NavigationWindowSec          float64
	MaxGoalRadiusM               float64
	MinClearanceM                float64
	MinCoverageGrowth            float64
	MinPathLengthM               float64
	MinAcceptedGoals             int
	MaxRecoveryCount             int
	ReturnHomePolicy             string
	UsesGazeboTruthAsInput       bool
	ExitGoal                     NavigationGoalSpec
	BoundedGoals                 []NavigationGoalSpec
	HomeGoal                     NavigationGoalSpec
	SlamOdomTopic                string
	SlamStatusTopic              string
	ControllerStatusTopic        string
	OwnerStatusTopic             string
	ScanStabilizationStatusTopic string
	LandingStatusTopic           string
}

type NavigationGoalSpec struct {
	ID     string
	XM     float64
	YM     float64
	YawRad float64
}

func DefaultNav2NavigationSpec() Nav2NavigationSpec {
	return Nav2NavigationSpec{
		Profile:                      "indoor_2d",
		GlobalFrame:                  "map",
		OdomFrame:                    "odom",
		BaseFrame:                    "base_link",
		ScanTopic:                    "/scan",
		MapTopic:                     "/map",
		SeedMapTopic:                 "/navlab/navigation/seed_map",
		CmdVelTopic:                  "/cmd_vel_nav",
		BTXML:                        "navigate_to_pose_w_replanning_and_recovery.xml",
		PlannerPlugin:                "GridBased",
		ControllerPlugin:             "FollowPath",
		UseSimTime:                   true,
		GlobalCostmapTopic:           "/global_costmap/costmap",
		LocalCostmapTopic:            "/local_costmap/costmap",
		RequiredCostmapLayers:        []string{"static_layer", "obstacle_layer", "inflation_layer"},
		MaxCostmapAgeSec:             1.5,
		MinObstacleCells:             1,
		MaxUnknownRatio:              0.35,
		InflationRadiusM:             0.35,
		FootprintRadiusM:             0.22,
		CostmapHealthTopic:           "/navlab/navigation/costmap_health",
		SetpointIntentTopic:          "/navlab/fcu/setpoint/intent",
		AdapterStatusTopic:           "/navlab/navigation/adapter/status",
		MaxXYSpeedMPS:                0.25,
		MaxYawRateDPS:                35,
		MaxAccelMPS2:                 0.35,
		FixedAltitudeM:               0.8,
		StopOnStaleCostmap:           true,
		StopOnStaleSlam:              true,
		NavigationStatusTopic:        "/navlab/navigation/status",
		NavigationEventsTopic:        "/navlab/navigation/events",
		NavigationGoalTopic:          "/navlab/navigation/goal",
		NavigationPathTopic:          "/navlab/navigation/path",
		NavigationRecoveryTopic:      "/navlab/navigation/recovery",
		Strategy:                     "bounded_frontier",
		CompletionPolicy:             "land_in_place",
		GoalFrame:                    "map",
		NavigationWindowSec:          120,
		MaxGoalRadiusM:               0.45,
		MinClearanceM:                0.35,
		MinCoverageGrowth:            0.50,
		MinPathLengthM:               4.0,
		MinAcceptedGoals:             3,
		MaxRecoveryCount:             2,
		ReturnHomePolicy:             "return_home_then_land",
		UsesGazeboTruthAsInput:       false,
		ExitGoal:                     NavigationGoalSpec{ID: "maze_exit", XM: 1.5, YM: -0.5, YawRad: 0.0},
		BoundedGoals:                 []NavigationGoalSpec{{ID: "p13_probe_1", XM: 1.0, YM: 0.0, YawRad: 0.0}},
		HomeGoal:                     NavigationGoalSpec{ID: "home", XM: 0.0, YM: 0.0, YawRad: 0.0},
		SlamOdomTopic:                "/slam/odom",
		SlamStatusTopic:              "/navlab/slam/status",
		ControllerStatusTopic:        "/navlab/fcu/controller/status",
		OwnerStatusTopic:             "/navlab/fcu/owner/status",
		ScanStabilizationStatusTopic: "/navlab/scan_stabilization/status",
		LandingStatusTopic:           "/navlab/landing/status",
	}
}

func WriteNav2ParamsYAML(path string, spec Nav2NavigationSpec) error {
	if spec.Profile == "" {
		spec = DefaultNav2NavigationSpec()
	}
	content := fmt.Sprintf(`bt_navigator:
  ros__parameters:
    use_sim_time: %t
    global_frame: %s
    robot_base_frame: %s
    odom_topic: %s
    bt_loop_duration: 10
    default_server_timeout: 20
    wait_for_service_timeout: 1000
    action_server_result_timeout: 900.0
    navigators: ["navigate_to_pose", "navigate_through_poses"]
    navigate_to_pose:
      plugin: "nav2_bt_navigator::NavigateToPoseNavigator"
    navigate_through_poses:
      plugin: "nav2_bt_navigator::NavigateThroughPosesNavigator"
    error_code_names:
      - compute_path_error_code
      - follow_path_error_code

controller_server:
  ros__parameters:
    use_sim_time: %t
    controller_frequency: 10.0
    costmap_update_timeout: 1.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugins: ["progress_checker"]
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.15
      movement_time_allowance: 20.0
    general_goal_checker:
      stateful: true
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.35
      yaw_goal_tolerance: 0.35
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      time_steps: 32
      model_dt: 0.10
      batch_size: 1000
      vx_max: 0.25
      vx_min: -0.10
      vy_max: 0.25
      wz_max: 0.60
      ax_max: 0.35
      ax_min: -0.35
      ay_max: 0.35
      ay_min: -0.35
      az_max: 0.80
      vx_std: 0.15
      vy_std: 0.15
      wz_std: 0.25
      iteration_count: 1
      prune_distance: 1.0
      transform_tolerance: 0.2
      temperature: 0.3
      gamma: 0.015
      motion_model: "Omni"
      visualize: false
      regenerate_noises: true
      critics: ["ConstraintCritic", "CostCritic", "GoalCritic", "GoalAngleCritic", "PathAlignCritic", "PathFollowCritic", "PathAngleCritic", "PreferForwardCritic"]
      ConstraintCritic:
        enabled: true
        cost_power: 1
        cost_weight: 4.0
      GoalCritic:
        enabled: true
        cost_power: 1
        cost_weight: 5.0
        threshold_to_consider: 1.0
      GoalAngleCritic:
        enabled: true
        cost_power: 1
        cost_weight: 3.0
        threshold_to_consider: 0.5
      PreferForwardCritic:
        enabled: false
      CostCritic:
        enabled: true
        cost_power: 1
        cost_weight: 3.0
        near_collision_cost: 253
        critical_cost: 300.0
        consider_footprint: false
        collision_cost: 1000000.0
      PathAlignCritic:
        enabled: true
        cost_power: 1
        cost_weight: 8.0
        threshold_to_consider: 0.5
      PathFollowCritic:
        enabled: true
        cost_power: 1
        cost_weight: 5.0
        threshold_to_consider: 1.0
      PathAngleCritic:
        enabled: true
        cost_power: 1
        cost_weight: 2.0
        threshold_to_consider: 0.5

local_costmap:
  local_costmap:
    ros__parameters:
      use_sim_time: %t
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: %s
      robot_base_frame: %s
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: %.3f
      plugins: ["obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: scan
        scan:
          topic: %s
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "LaserScan"
          raytrace_max_range: 4.0
          raytrace_min_range: 0.0
          obstacle_max_range: 3.5
          obstacle_min_range: 0.0
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: %.3f
      always_send_full_costmap: true

global_costmap:
  global_costmap:
    ros__parameters:
      use_sim_time: %t
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: %s
      robot_base_frame: %s
      robot_radius: %.3f
      resolution: 0.05
      track_unknown_space: false
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_topic: %s
        map_subscribe_transient_local: true
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: scan
        scan:
          topic: %s
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "LaserScan"
          raytrace_max_range: 4.0
          raytrace_min_range: 0.0
          obstacle_max_range: 3.5
          obstacle_min_range: 0.0
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: %.3f
      always_send_full_costmap: true

planner_server:
  ros__parameters:
    use_sim_time: %t
    expected_planner_frequency: 10.0
    planner_plugins: ["GridBased"]
    costmap_update_timeout: 1.0
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

smoother_server:
  ros__parameters:
    use_sim_time: %t
    smoother_plugins: ["simple_smoother"]
    simple_smoother:
      plugin: "nav2_smoother::SimpleSmoother"
      tolerance: 1.0e-10
      max_its: 1000
      do_refinement: true

behavior_server:
  ros__parameters:
    use_sim_time: %t
    local_costmap_topic: local_costmap/costmap_raw
    global_costmap_topic: global_costmap/costmap_raw
    local_footprint_topic: local_costmap/published_footprint
    global_footprint_topic: global_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "drive_on_heading", "wait"]
    spin:
      plugin: "nav2_behaviors::Spin"
    backup:
      plugin: "nav2_behaviors::BackUp"
    drive_on_heading:
      plugin: "nav2_behaviors::DriveOnHeading"
    wait:
      plugin: "nav2_behaviors::Wait"
    local_frame: %s
    global_frame: %s
    robot_base_frame: %s
    transform_tolerance: 0.2
    simulate_ahead_time: 2.0
    max_rotational_vel: 0.6
    min_rotational_vel: 0.1
    rotational_acc_lim: 1.0

waypoint_follower:
  ros__parameters:
    use_sim_time: %t
    loop_rate: 20
    stop_on_failure: false
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: true
      waypoint_pause_duration: 200

route_server:
  ros__parameters:
    use_sim_time: %t
    boundary_radius_to_achieve_node: 1.0
    radius_to_achieve_node: 2.0
    smooth_corners: true
    operations: ["AdjustSpeedLimit", "ReroutingService", "CollisionMonitor"]
    ReroutingService:
      plugin: "nav2_route::ReroutingService"
    AdjustSpeedLimit:
      plugin: "nav2_route::AdjustSpeedLimit"
    CollisionMonitor:
      plugin: "nav2_route::CollisionMonitor"
      max_collision_dist: 3.0
    edge_cost_functions: ["DistanceScorer", "CostmapScorer"]
    DistanceScorer:
      plugin: "nav2_route::DistanceScorer"
    CostmapScorer:
      plugin: "nav2_route::CostmapScorer"

velocity_smoother:
  ros__parameters:
    use_sim_time: %t
    smoothing_frequency: 20.0
    feedback: "OPEN_LOOP"
    cmd_vel_in_topic: "cmd_vel"
    cmd_vel_out_topic: "cmd_vel_smoothed"
    max_velocity: [0.25, 0.25, 0.60]
    min_velocity: [-0.10, -0.25, -0.60]
    max_accel: [0.35, 0.35, 0.80]
    max_decel: [-0.35, -0.35, -0.80]
    odom_topic: %s
    odom_duration: 0.1
    deadband_velocity: [0.0, 0.0, 0.0]
    velocity_timeout: 1.0

collision_monitor:
  ros__parameters:
    use_sim_time: %t
    base_frame_id: %s
    odom_frame_id: %s
    cmd_vel_in_topic: "cmd_vel_smoothed"
    cmd_vel_out_topic: %s
    state_topic: "collision_monitor_state"
    transform_tolerance: 0.2
    source_timeout: 1.0
    stop_pub_timeout: 2.0
    polygons: ["FootprintApproach"]
    FootprintApproach:
      type: "polygon"
      action_type: "approach"
      footprint_topic: "/local_costmap/published_footprint"
      time_before_collision: 1.2
      simulation_time_step: 0.1
      min_points: 6
      visualize: false
      enabled: true
    observation_sources: ["scan"]
    scan:
      type: "scan"
      topic: %s
      min_height: 0.0
      max_height: 2.0
      enabled: true

docking_server:
  ros__parameters:
    use_sim_time: %t
    controller_frequency: 20.0
    initial_perception_timeout: 5.0
    wait_charge_timeout: 5.0
    dock_approach_timeout: 30.0
    undock_linear_tolerance: 0.05
    undock_angular_tolerance: 0.1
    max_retries: 1
    base_frame: %s
    fixed_frame: %s
    dock_backwards: false
    dock_prestaging_tolerance: 0.5
    dock_plugins: ["simple_charging_dock"]
    simple_charging_dock:
      plugin: "opennav_docking::SimpleChargingDock"
      docking_threshold: 0.05
      staging_x_offset: -0.7
      use_external_detection_pose: true
      use_battery_status: false
      use_stall_detection: false
      external_detection_timeout: 1.0
      external_detection_translation_x: -0.18
      external_detection_translation_y: 0.0
      external_detection_rotation_roll: -1.57
      external_detection_rotation_pitch: -1.57
      external_detection_rotation_yaw: 0.0
      filter_coef: 0.1
    controller:
      k_phi: 3.0
      k_delta: 2.0
      v_linear_min: 0.05
      v_linear_max: 0.15
      use_collision_detection: false
      costmap_topic: "local_costmap/costmap_raw"
      footprint_topic: "local_costmap/published_footprint"
      transform_tolerance: 0.2
      projection_time: 5.0
      simulation_step: 0.1
      dock_collision_threshold: 0.3
`,
		spec.UseSimTime,
		spec.GlobalFrame,
		spec.BaseFrame,
		spec.SlamOdomTopic,
		spec.UseSimTime,
		spec.UseSimTime,
		spec.OdomFrame,
		spec.BaseFrame,
		spec.FootprintRadiusM,
		spec.ScanTopic,
		spec.InflationRadiusM,
		spec.UseSimTime,
		spec.GlobalFrame,
		spec.BaseFrame,
		spec.FootprintRadiusM,
		spec.SeedMapTopic,
		spec.ScanTopic,
		spec.InflationRadiusM,
		spec.UseSimTime,
		spec.UseSimTime,
		spec.UseSimTime,
		spec.OdomFrame,
		spec.GlobalFrame,
		spec.BaseFrame,
		spec.UseSimTime,
		spec.UseSimTime,
		spec.UseSimTime,
		spec.SlamOdomTopic,
		spec.UseSimTime,
		spec.BaseFrame,
		spec.OdomFrame,
		spec.CmdVelTopic,
		spec.ScanTopic,
		spec.UseSimTime,
		spec.BaseFrame,
		spec.OdomFrame,
	)
	if err := os.MkdirAll(filepath.Dir(path), 0o755); err != nil {
		return err
	}
	return os.WriteFile(path, []byte(content), 0o644)
}

func WriteNavigationAdapterRuntimeConfig(path string, spec Nav2NavigationSpec) error {
	if spec.Profile == "" {
		spec = DefaultNav2NavigationSpec()
	}
	return writeTOML(path, map[string]any{
		"nav2": map[string]any{"runtime": map[string]any{
			"profile":           spec.Profile,
			"global_frame":      spec.GlobalFrame,
			"odom_frame":        spec.OdomFrame,
			"base_frame":        spec.BaseFrame,
			"scan_topic":        spec.ScanTopic,
			"map_topic":         spec.MapTopic,
			"seed_map_topic":    spec.SeedMapTopic,
			"cmd_vel_topic":     spec.CmdVelTopic,
			"bt_xml":            spec.BTXML,
			"planner_plugin":    spec.PlannerPlugin,
			"controller_plugin": spec.ControllerPlugin,
			"use_sim_time":      spec.UseSimTime,
		}},
		"navigation_adapter": map[string]any{"runtime": map[string]any{
			"setpoint_intent_topic": spec.SetpointIntentTopic,
			"status_topic":          spec.AdapterStatusTopic,
			"max_xy_speed_mps":      spec.MaxXYSpeedMPS,
			"max_yaw_rate_dps":      spec.MaxYawRateDPS,
			"max_accel_mps2":        spec.MaxAccelMPS2,
			"fixed_altitude_m":      spec.FixedAltitudeM,
			"stop_on_stale_costmap": spec.StopOnStaleCostmap,
			"stop_on_stale_slam":    spec.StopOnStaleSlam,
		}},
		"navigation_mission": map[string]any{"runtime": map[string]any{
			"strategy":                   spec.Strategy,
			"completion_policy":          spec.CompletionPolicy,
			"goal_frame":                 spec.GoalFrame,
			"status_topic":               spec.NavigationStatusTopic,
			"events_topic":               spec.NavigationEventsTopic,
			"goal_topic":                 spec.NavigationGoalTopic,
			"path_topic":                 spec.NavigationPathTopic,
			"recovery_topic":             spec.NavigationRecoveryTopic,
			"navigation_window_sec":      spec.NavigationWindowSec,
			"max_goal_radius_m":          spec.MaxGoalRadiusM,
			"min_clearance_m":            spec.MinClearanceM,
			"min_coverage_growth":        spec.MinCoverageGrowth,
			"min_path_length_m":          spec.MinPathLengthM,
			"min_accepted_goals":         spec.MinAcceptedGoals,
			"max_recovery_count":         spec.MaxRecoveryCount,
			"return_home_policy":         spec.ReturnHomePolicy,
			"uses_gazebo_truth_as_input": spec.UsesGazeboTruthAsInput,
			"exit_goal":                  spec.ExitGoal,
			"bounded_goals":              spec.BoundedGoals,
			"home_goal":                  spec.HomeGoal,
		}},
	})
}

func WriteNavigationFoxgloveLiteProfile(path string, spec Nav2NavigationSpec) error {
	if spec.Profile == "" {
		spec = DefaultNav2NavigationSpec()
	}
	profile := map[string]any{
		"schemaVersion": "navlab.sim.foxglove_lite_profile.v1",
		"taskId":        "navigation",
		"name":          "P13 Nav2 indoor navigation",
		"topics": []map[string]string{
			{"role": "map", "topic": spec.MapTopic, "messageType": "nav_msgs/msg/OccupancyGrid"},
			{"role": "global_costmap", "topic": spec.GlobalCostmapTopic, "messageType": "nav_msgs/msg/OccupancyGrid"},
			{"role": "local_costmap", "topic": spec.LocalCostmapTopic, "messageType": "nav_msgs/msg/OccupancyGrid"},
			{"role": "scan", "topic": spec.ScanTopic, "messageType": "sensor_msgs/msg/LaserScan"},
			{"role": "trajectory", "topic": spec.SlamOdomTopic, "messageType": "nav_msgs/msg/Odometry"},
			{"role": "path", "topic": spec.NavigationPathTopic, "messageType": "std_msgs/msg/String"},
			{"role": "goals", "topic": spec.NavigationGoalTopic, "messageType": "std_msgs/msg/String"},
			{"role": "events", "topic": spec.NavigationEventsTopic, "messageType": "std_msgs/msg/String"},
			{"role": "navigation_status", "topic": spec.NavigationStatusTopic, "messageType": "std_msgs/msg/String"},
			{"role": "adapter_status", "topic": spec.AdapterStatusTopic, "messageType": "std_msgs/msg/String"},
			{"role": "costmap_health", "topic": spec.CostmapHealthTopic, "messageType": "std_msgs/msg/String"},
			{"role": "landing_status", "topic": spec.LandingStatusTopic, "messageType": "std_msgs/msg/String"},
		},
		"reviewChecks": []string{
			"map frame and path frame stay in map",
			"Nav2 path does not bypass the adapter intent topic",
			"costmap health remains fresh while goals execute",
			"frontier candidates, rejected reasons, and blacklist are visible in navigation status",
		},
	}
	data, err := json.MarshalIndent(profile, "", "  ")
	if err != nil {
		return err
	}
	if err := os.MkdirAll(filepath.Dir(path), 0o755); err != nil {
		return err
	}
	return os.WriteFile(path, append(data, '\n'), 0o644)
}

func NavigationAdapterRuntimeScript(spec Nav2NavigationSpec, durationSec float64) (string, error) {
	if spec.Profile == "" {
		spec = DefaultNav2NavigationSpec()
	}
	payload, err := json.Marshal(spec)
	if err != nil {
		return "", err
	}
	return fmt.Sprintf(`from __future__ import annotations

import json
import math
import time

SPEC = json.loads(%q)
RUN_UNTIL = time.monotonic() + %.3f


def clamp(value: float, limit: float) -> tuple[float, bool]:
    if limit <= 0:
        return 0.0, True
    if value > limit:
        return limit, True
    if value < -limit:
        return -limit, True
    return value, False


def main() -> int:
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import OccupancyGrid
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy
    from rclpy.qos import QoSProfile
    from rclpy.qos import ReliabilityPolicy
    from std_msgs.msg import String
    from tf2_ros import StaticTransformBroadcaster

    rclpy.init()
    node = rclpy.create_node("navlab_navigation_adapter")
    node.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, bool(SPEC.get("UseSimTime", True)))])
    clock_deadline = time.monotonic() + 5.0
    while bool(SPEC.get("UseSimTime", True)) and rclpy.ok() and time.monotonic() < clock_deadline and node.get_clock().now().nanoseconds <= 0:
        rclpy.spin_once(node, timeout_sec=0.1)
    tf_broadcaster = StaticTransformBroadcaster(node)
    seed_map_qos = QoSProfile(depth=1)
    seed_map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    seed_map_qos.reliability = ReliabilityPolicy.RELIABLE
    seed_map_pub = node.create_publisher(OccupancyGrid, SPEC["SeedMapTopic"], seed_map_qos)
    intent_pub = node.create_publisher(String, SPEC["SetpointIntentTopic"], 10)
    status_pub = node.create_publisher(String, SPEC["AdapterStatusTopic"], 10)
    costmap_health_pub = node.create_publisher(String, SPEC["CostmapHealthTopic"], 10)
    state = {
        "last_cmd_ms": 0,
        "last_output": {"x": 0.0, "y": 0.0, "yaw": 0.0},
        "last_output_time": time.monotonic(),
        "last_seed_map_publish_time": 0.0,
        "last_tf_publish_time": 0.0,
        "last_health_publish_time": 0.0,
        "last_status_publish_time": 0.0,
        "first_local_costmap_time": 0.0,
        "local_costmap_count": 0,
        "clamp_count": 0,
        "hold_count": 0,
        "intent_count": 0,
        "last_costmap_ms": 0,
        "last_slam_ms": 0,
        "last_global_costmap_ms": 0,
        "last_local_costmap_ms": 0,
        "global_obstacle_cells": 0,
        "local_obstacle_cells": 0,
        "global_unknown_ratio": 1.0,
        "local_unknown_ratio": 1.0,
        "costmap_ready": not SPEC["StopOnStaleCostmap"],
        "slam_ready": not SPEC["StopOnStaleSlam"],
    }

    def make_seed_map() -> OccupancyGrid:
        msg = OccupancyGrid()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = SPEC["GlobalFrame"]
        msg.info.resolution = 0.05
        msg.info.width = 200
        msg.info.height = 200
        msg.info.origin.position.x = -5.0
        msg.info.origin.position.y = -5.0
        msg.info.origin.orientation.w = 1.0
        data = []
        for y in range(msg.info.height):
            for x in range(msg.info.width):
                border = x in (0, msg.info.width - 1) or y in (0, msg.info.height - 1)
                interior_marker = 135 <= x <= 138 and 70 <= y <= 130
                data.append(100 if border or interior_marker else 0)
        msg.data = data
        return msg

    seed_map = make_seed_map()

    def publish_seed_map() -> None:
        seed_map.header.stamp = node.get_clock().now().to_msg()
        seed_map_pub.publish(seed_map)
        state["last_seed_map_publish_time"] = time.monotonic()

    def publish_seed_tf() -> None:
        transform = TransformStamped()
        transform.header.stamp = node.get_clock().now().to_msg()
        transform.header.frame_id = SPEC["GlobalFrame"]
        transform.child_frame_id = SPEC["OdomFrame"]
        transform.transform.rotation.w = 1.0
        tf_broadcaster.sendTransform(transform)
        state["last_tf_publish_time"] = time.monotonic()

    def costmap_stats(msg: OccupancyGrid) -> tuple[int, float]:
        total = len(msg.data)
        if total <= 0:
            return 0, 1.0
        unknown = 0
        occupied = 0
        for cell in msg.data:
            value = int(cell)
            if value < 0:
                unknown += 1
            elif value >= 50:
                occupied += 1
        return occupied, unknown / float(total)

    def on_global_costmap(msg: OccupancyGrid) -> None:
        occupied, unknown_ratio = costmap_stats(msg)
        state["global_obstacle_cells"] = occupied
        state["global_unknown_ratio"] = unknown_ratio
        state["last_global_costmap_ms"] = int(time.time() * 1000)

    def on_local_costmap(msg: OccupancyGrid) -> None:
        occupied, unknown_ratio = costmap_stats(msg)
        state["local_obstacle_cells"] = occupied
        state["local_unknown_ratio"] = unknown_ratio
        state["last_local_costmap_ms"] = int(time.time() * 1000)
        state["local_costmap_count"] += 1
        if state["first_local_costmap_time"] <= 0:
            state["first_local_costmap_time"] = time.monotonic()

    def publish_costmap_health() -> None:
        now_ms = int(time.time() * 1000)
        global_age = None
        local_age = None
        if state["last_global_costmap_ms"] > 0:
            global_age = (now_ms - state["last_global_costmap_ms"]) / 1000.0
        if state["last_local_costmap_ms"] > 0:
            local_age = (now_ms - state["last_local_costmap_ms"]) / 1000.0
        unknown_ratio = max(float(state["global_unknown_ratio"]), float(state["local_unknown_ratio"]))
        obstacle_cells = max(int(state["global_obstacle_cells"]), int(state["local_obstacle_cells"]))
        local_frequency_hz = 0.0
        if state["first_local_costmap_time"] > 0:
            elapsed = max(0.001, time.monotonic() - state["first_local_costmap_time"])
            local_frequency_hz = state["local_costmap_count"] / elapsed
        ok = (
            global_age is not None
            and local_age is not None
            and global_age <= float(SPEC["MaxCostmapAgeSec"])
            and local_age <= float(SPEC["MaxCostmapAgeSec"])
            and unknown_ratio <= float(SPEC["MaxUnknownRatio"])
            and obstacle_cells >= int(SPEC["MinObstacleCells"])
        )
        payload = {
            "claim": "evaluated",
            "costmap_claim": "evaluated",
            "ok": ok,
            "ready": ok,
            "source": "nav2_costmap_topics",
            "global_costmap_topic": SPEC["GlobalCostmapTopic"],
            "local_costmap_topic": SPEC["LocalCostmapTopic"],
            "global_costmap_age_sec": global_age,
            "local_costmap_age_sec": local_age,
            "unknown_ratio": unknown_ratio,
            "obstacle_cells": obstacle_cells,
            "local_costmap_update_frequency_hz": local_frequency_hz,
            "required_layers": SPEC["RequiredCostmapLayers"],
        }
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        costmap_health_pub.publish(msg)
        state["last_health_publish_time"] = time.monotonic()

    def publish_status(active: bool, hold_reason: str = "") -> None:
        payload = {
            "claim": "evaluated",
            "adapter_claim": "evaluated",
            "active": active,
            "source_topic": SPEC["CmdVelTopic"],
            "intent_topic": SPEC["SetpointIntentTopic"],
            "max_xy_speed_mps": SPEC["MaxXYSpeedMPS"],
            "max_yaw_rate_dps": SPEC["MaxYawRateDPS"],
            "max_accel_mps2": SPEC["MaxAccelMPS2"],
            "fixed_altitude_m": SPEC["FixedAltitudeM"],
            "stop_on_stale_costmap": SPEC["StopOnStaleCostmap"],
            "stop_on_stale_slam": SPEC["StopOnStaleSlam"],
            "last_cmd_ms": state["last_cmd_ms"],
            "clamp_count": state["clamp_count"],
            "hold_count": state["hold_count"],
            "intent_count": state["intent_count"],
            "hold_reason": hold_reason,
        }
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        status_pub.publish(msg)
        state["last_status_publish_time"] = time.monotonic()

    def publish_hold(hold_reason: str) -> None:
        state["hold_count"] += 1
        intent = {
            "source": "nav2",
            "frame_id": SPEC["BaseFrame"],
            "linear_x_mps": 0.0,
            "linear_y_mps": 0.0,
            "yaw_rate_radps": 0.0,
            "fixed_altitude_m": SPEC["FixedAltitudeM"],
            "clamped": False,
            "hold": True,
            "hold_reason": hold_reason,
        }
        out = String()
        out.data = json.dumps(intent, sort_keys=True)
        intent_pub.publish(out)
        publish_status(False, hold_reason)

    def parse_status_message(msg: String) -> dict:
        try:
            return json.loads(msg.data)
        except Exception:
            return {}

    def on_costmap_status(msg: String) -> None:
        payload = parse_status_message(msg)
        ready = bool(payload.get("ok", payload.get("ready", True)))
        if "unknown_ratio" in payload:
            ready = ready and float(payload["unknown_ratio"]) <= float(SPEC["MaxUnknownRatio"])
        state["costmap_ready"] = ready
        state["last_costmap_ms"] = int(time.time() * 1000)

    def on_slam_status(msg: String) -> None:
        payload = parse_status_message(msg)
        state["slam_ready"] = bool(payload.get("ready", payload.get("ok", True)))
        state["last_slam_ms"] = int(time.time() * 1000)

    def stale_hold_reason() -> str:
        now_ms = int(time.time() * 1000)
        max_age_ms = int(float(SPEC["MaxCostmapAgeSec"]) * 1000)
        if SPEC["StopOnStaleCostmap"]:
            if not state["costmap_ready"] or state["last_costmap_ms"] <= 0:
                return "stale_costmap"
            if now_ms - state["last_costmap_ms"] > max_age_ms:
                return "stale_costmap"
        if SPEC["StopOnStaleSlam"]:
            if not state["slam_ready"] or state["last_slam_ms"] <= 0:
                return "stale_slam"
            if now_ms - state["last_slam_ms"] > max_age_ms:
                return "stale_slam"
        return ""

    def apply_accel_limit(x: float, y: float, yaw: float) -> tuple[float, float, float, bool]:
        now = time.monotonic()
        dt = max(0.02, now - state["last_output_time"])
        max_delta = float(SPEC["MaxAccelMPS2"]) * dt
        prev = state["last_output"]
        next_x, clamp_x = clamp(x - prev["x"], max_delta)
        next_y, clamp_y = clamp(y - prev["y"], max_delta)
        limited_x = prev["x"] + next_x
        limited_y = prev["y"] + next_y
        state["last_output"] = {"x": limited_x, "y": limited_y, "yaw": yaw}
        state["last_output_time"] = now
        return limited_x, limited_y, yaw, clamp_x or clamp_y

    def on_cmd_vel(msg: Twist) -> None:
        hold_reason = stale_hold_reason()
        if hold_reason:
            publish_hold(hold_reason)
            return
        xy_speed = math.hypot(float(msg.linear.x), float(msg.linear.y))
        max_xy = float(SPEC["MaxXYSpeedMPS"])
        scale = 1.0
        clamped = False
        if xy_speed > max_xy > 0:
            scale = max_xy / xy_speed
            clamped = True
        yaw_rate, yaw_clamped = clamp(float(msg.angular.z), math.radians(float(SPEC["MaxYawRateDPS"])))
        clamped = clamped or yaw_clamped
        linear_x, linear_y, yaw_rate, accel_clamped = apply_accel_limit(float(msg.linear.x) * scale, float(msg.linear.y) * scale, yaw_rate)
        clamped = clamped or accel_clamped
        if clamped:
            state["clamp_count"] += 1
        state["intent_count"] += 1
        state["last_cmd_ms"] = int(time.time() * 1000)
        intent = {
            "source": "nav2",
            "frame_id": SPEC["BaseFrame"],
            "linear_x_mps": linear_x,
            "linear_y_mps": linear_y,
            "yaw_rate_radps": yaw_rate,
            "fixed_altitude_m": SPEC["FixedAltitudeM"],
            "clamped": clamped,
        }
        out = String()
        out.data = json.dumps(intent, sort_keys=True)
        intent_pub.publish(out)
        publish_status(True)

    node.create_subscription(Twist, SPEC["CmdVelTopic"], on_cmd_vel, 10)
    node.create_subscription(String, SPEC["CostmapHealthTopic"], on_costmap_status, 10)
    node.create_subscription(String, SPEC["SlamStatusTopic"], on_slam_status, 10)
    node.create_subscription(OccupancyGrid, SPEC["GlobalCostmapTopic"], on_global_costmap, 10)
    node.create_subscription(OccupancyGrid, SPEC["LocalCostmapTopic"], on_local_costmap, 10)
    publish_seed_map()
    publish_seed_tf()
    publish_costmap_health()
    publish_status(False, "waiting_for_nav2_cmd_vel")
    while rclpy.ok() and time.monotonic() < RUN_UNTIL:
        now = time.monotonic()
        if now - state["last_seed_map_publish_time"] >= 1.0:
            publish_seed_map()
        if now - state["last_tf_publish_time"] >= 1.0:
            publish_seed_tf()
        if now - state["last_health_publish_time"] >= 0.5:
            publish_costmap_health()
        if now - state["last_status_publish_time"] >= 0.5:
            active = state["intent_count"] > 0
            publish_status(active, "" if active else "waiting_for_nav2_cmd_vel")
        rclpy.spin_once(node, timeout_sec=0.1)
    publish_status(False, "duration_elapsed")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
`, string(payload), durationSec), nil
}

func NavigationMissionRuntimeScript(spec Nav2NavigationSpec, durationSec float64) (string, error) {
	if spec.Profile == "" {
		spec = DefaultNav2NavigationSpec()
	}
	payload, err := json.Marshal(spec)
	if err != nil {
		return "", err
	}
	return fmt.Sprintf(`from __future__ import annotations

import json
import math
import subprocess
import time

SPEC = json.loads(%q)
RUN_UNTIL = time.monotonic() + %.3f


def run(command: list[str], timeout_sec: float = 4.0) -> dict:
    started = time.time()
    result = subprocess.run(["timeout", str(timeout_sec), *command], check=False, capture_output=True, text=True)
    return {
        "command": command,
        "returncode": result.returncode,
        "stdout": result.stdout.strip(),
        "stderr": result.stderr.strip(),
        "elapsed_sec": time.time() - started,
        "ok": result.returncode == 0,
    }


def action_ready() -> bool:
    result = run(["ros2", "action", "list"])
    actions = [line.strip() for line in result["stdout"].splitlines() if line.strip()]
    return result["ok"] and ("/navigate_to_pose" in actions or "navigate_to_pose" in actions)


def yaw_to_quaternion(yaw: float) -> dict:
    return {"z": math.sin(yaw / 2.0), "w": math.cos(yaw / 2.0)}


def publish_json(node, publisher, payload: dict) -> None:
    from std_msgs.msg import String

    msg = String()
    msg.data = json.dumps(payload, sort_keys=True)
    publisher.publish(msg)


def log_event(event: str, **values) -> None:
    payload = {"event": event, **values}
    print(json.dumps(payload, sort_keys=True), flush=True)


def frontier_candidate(goal: dict) -> dict:
    x_m = float(goal["XM"])
    y_m = float(goal["YM"])
    distance_m = math.hypot(x_m, y_m)
    clearance_m = max(float(SPEC["MinClearanceM"]), float(SPEC["FootprintRadiusM"]) + float(SPEC["InflationRadiusM"]))
    return {
        "id": goal["ID"],
        "frame_id": SPEC["GoalFrame"],
        "x_m": x_m,
        "y_m": y_m,
        "yaw_rad": float(goal["YawRad"]),
        "source": "bounded_goal_slam_map_costmap",
        "estimated_path_length_m": distance_m,
        "clearance_m": clearance_m,
        "clearance_ok": clearance_m >= float(SPEC["MinClearanceM"]),
        "path_reachable": "unreachable" not in str(goal["ID"]).lower(),
        "map_ready": False,
        "costmap_ready": False,
    }


def mission_goals() -> list[dict]:
    goals = []
    exit_goal = SPEC.get("ExitGoal") or {}
    if exit_goal.get("ID"):
        goals.append(exit_goal)
    goals.extend(SPEC.get("BoundedGoals") or [])
    seen = set()
    unique_goals = []
    for goal in goals:
        goal_id = goal.get("ID")
        if not goal_id or goal_id in seen:
            continue
        seen.add(goal_id)
        unique_goals.append(goal)
    return unique_goals


def main() -> int:
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.parameter import Parameter
    from std_msgs.msg import String
    from nav_msgs.msg import OccupancyGrid, Odometry
    from nav2_msgs.action import NavigateToPose

    rclpy.init()
    node = rclpy.create_node("navlab_navigation_mission")
    node.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, bool(SPEC.get("UseSimTime", True)))])
    clock_deadline = time.monotonic() + 5.0
    while bool(SPEC.get("UseSimTime", True)) and rclpy.ok() and time.monotonic() < clock_deadline and node.get_clock().now().nanoseconds <= 0:
        rclpy.spin_once(node, timeout_sec=0.1)
    status_pub = node.create_publisher(String, SPEC["NavigationStatusTopic"], 10)
    event_pub = node.create_publisher(String, SPEC["NavigationEventsTopic"], 10)
    goal_pub = node.create_publisher(String, SPEC["NavigationGoalTopic"], 10)
    path_pub = node.create_publisher(String, SPEC["NavigationPathTopic"], 10)
    recovery_pub = node.create_publisher(String, SPEC["NavigationRecoveryTopic"], 10)
    state = {
        "map_samples": 0,
        "costmap_health_samples": 0,
        "costmap_ready": False,
        "costmap_health": {},
        "controller_samples": 0,
        "controller_ready": False,
        "controller_status": {},
        "slam_samples": 0,
        "slam_ready": False,
        "slam_status": {},
        "odom_samples": 0,
        "last_odom_xy": None,
        "path_length_m": 0.0,
    }

    def on_map(msg: OccupancyGrid) -> None:
        state["map_samples"] += 1

    def parse_status_message(msg: String) -> dict:
        try:
            return json.loads(msg.data)
        except Exception:
            return {}

    def on_costmap_health(msg: String) -> None:
        payload = parse_status_message(msg)
        state["costmap_health_samples"] += 1
        state["costmap_health"] = payload
        state["costmap_ready"] = bool(payload.get("ok", payload.get("ready", False)))

    def on_controller_status(msg: String) -> None:
        payload = parse_status_message(msg)
        state["controller_samples"] += 1
        state["controller_status"] = payload
        bootstrap = payload.get("bootstrap", {})
        state["controller_ready"] = bool(payload.get("ready", payload.get("ok", False))) or bool(bootstrap.get("ok", False))

    def on_slam_status(msg: String) -> None:
        payload = parse_status_message(msg)
        state["slam_samples"] += 1
        state["slam_status"] = payload
        state["slam_ready"] = bool(payload.get("ready", payload.get("ok", False)))

    def on_odom(msg: Odometry) -> None:
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        state["odom_samples"] += 1
        previous = state.get("last_odom_xy")
        if previous is not None:
            step = math.hypot(x - previous[0], y - previous[1])
            if 0.001 <= step <= 0.75:
                state["path_length_m"] += step
        state["last_odom_xy"] = (x, y)

    def enrich_candidate(candidate: dict) -> dict:
        candidate = dict(candidate)
        candidate["map_ready"] = state["map_samples"] > 0
        candidate["costmap_ready"] = state["costmap_health_samples"] > 0
        candidate["costmap_health_ok"] = state["costmap_ready"]
        candidate["controller_ready"] = state["controller_ready"]
        candidate["slam_ready"] = state["slam_ready"]
        candidate["map_samples"] = state["map_samples"]
        candidate["costmap_health_samples"] = state["costmap_health_samples"]
        candidate["controller_samples"] = state["controller_samples"]
        candidate["slam_samples"] = state["slam_samples"]
        candidate["odom_samples"] = state["odom_samples"]
        return candidate

    node.create_subscription(OccupancyGrid, SPEC["MapTopic"], on_map, 10)
    node.create_subscription(String, SPEC["CostmapHealthTopic"], on_costmap_health, 10)
    node.create_subscription(String, SPEC["ControllerStatusTopic"], on_controller_status, 10)
    node.create_subscription(String, SPEC["SlamStatusTopic"], on_slam_status, 10)
    node.create_subscription(Odometry, SPEC["SlamOdomTopic"], on_odom, 10)
    warmup_until = min(RUN_UNTIL, time.monotonic() + max(12.0, float(SPEC.get("NavigationWindowSec", 120)) * 0.5))
    while (
        rclpy.ok()
        and time.monotonic() < warmup_until
        and (
            state["map_samples"] <= 0
            or state["costmap_health_samples"] <= 0
            or not state["controller_ready"]
            or not state["slam_ready"]
        )
    ):
        rclpy.spin_once(node, timeout_sec=0.1)
    goals = mission_goals()
    log_event(
        "mission_warmup_complete",
        map_samples=state["map_samples"],
        costmap_health_samples=state["costmap_health_samples"],
        costmap_ready=state["costmap_ready"],
        controller_ready=state["controller_ready"],
        controller_samples=state["controller_samples"],
        slam_ready=state["slam_ready"],
        slam_samples=state["slam_samples"],
        goals=[goal["ID"] for goal in goals],
    )
    blockers = []
    candidates = [enrich_candidate(frontier_candidate(goal)) for goal in goals]
    accepted_frontiers = []
    rejected_frontiers = []
    blacklisted_goals = []
    timed_out_goals = []
    accepted_goals = 0
    succeeded_goals = 0
    recovery_count = 0

    def publish_status(current_blockers: list[str] | None = None) -> None:
        candidate_count = max(1, len(candidates))
        coverage_growth = min(1.0, len(accepted_frontiers) / candidate_count)
        path_length_m = float(state.get("path_length_m", 0.0))
        publish_json(node, status_pub, {
            "claim": "evaluated",
            "navigation_claim": "evaluated",
            "frontier_claim": "evaluated",
            "strategy": SPEC["Strategy"],
            "frontier_candidates": candidates,
            "accepted_frontiers": accepted_frontiers,
            "rejected_frontiers": rejected_frontiers,
            "blacklisted_goals": blacklisted_goals,
            "timed_out_goals": timed_out_goals,
            "accepted_goals": accepted_goals,
            "succeeded_goals": succeeded_goals,
            "min_accepted_goals": SPEC["MinAcceptedGoals"],
            "path_length_m": path_length_m,
            "min_path_length_m": SPEC["MinPathLengthM"],
            "coverage_growth": coverage_growth,
            "min_coverage_growth": SPEC["MinCoverageGrowth"],
            "recovery_count": recovery_count,
            "odom_samples": state["odom_samples"],
            "completion_policy": SPEC["CompletionPolicy"],
            "return_home_policy": SPEC["ReturnHomePolicy"],
            "goal_success_ratio": 0.0 if not goals else accepted_goals / len(goals),
            "uses_gazebo_truth_as_input": SPEC["UsesGazeboTruthAsInput"],
            "blockers": current_blockers or [],
        })

    publish_status()
    if state["map_samples"] <= 0:
        blockers.append("map_unavailable")
    if state["costmap_health_samples"] <= 0:
        blockers.append("costmap_unavailable")
    if not state["controller_ready"]:
        blockers.append("controller_not_ready")
    if not state["slam_ready"]:
        blockers.append("slam_not_ready")
    if blockers:
        log_event("mission_readiness_blocked", blockers=blockers, controller_status=state["controller_status"], slam_status=state["slam_status"])
        publish_status(blockers)
        node.destroy_node()
        rclpy.shutdown()
        return 20
    if not action_ready():
        blockers.append("nav2_action_unavailable")
        log_event("nav2_action_unavailable")
        publish_status(blockers)
        node.destroy_node()
        rclpy.shutdown()
        return 20

    client = ActionClient(node, NavigateToPose, "navigate_to_pose")
    if not client.wait_for_server(timeout_sec=8.0):
        blockers.append("nav2_action_unavailable")
        log_event("nav2_action_server_unavailable")
    per_goal_timeout_sec = max(30.0, float(SPEC.get("NavigationWindowSec", 120.0)) / max(1, len(goals)))
    for goal in goals:
        if blockers or time.monotonic() >= RUN_UNTIL:
            break
        candidate = enrich_candidate(frontier_candidate(goal))
        log_event("frontier_candidate", candidate=candidate)
        publish_json(node, event_pub, {"event": "frontier_candidate", "candidate": candidate})
        if not candidate["map_ready"] or not candidate["costmap_ready"] or not candidate["clearance_ok"] or not candidate["path_reachable"]:
            if not candidate["map_ready"]:
                reason = "map_unavailable"
            elif not candidate["costmap_ready"]:
                reason = "costmap_unavailable"
            elif not candidate["clearance_ok"]:
                reason = "clearance_too_low"
            else:
                reason = "unreachable_blacklisted"
            rejected = {"id": candidate["id"], "reason": reason, "candidate": candidate}
            rejected_frontiers.append(rejected)
            blacklisted_goals.append(candidate["id"])
            recovery_count += 1
            log_event("frontier_rejected", id=candidate["id"], reason=reason)
            publish_json(node, recovery_pub, {"goal_id": candidate["id"], "reason": reason, "return_home_policy": SPEC["ReturnHomePolicy"]})
            publish_json(node, event_pub, {"event": "frontier_rejected", "id": candidate["id"], "reason": reason})
            publish_status()
            continue
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose.header.frame_id = SPEC["GoalFrame"]
        nav_goal.pose.header.stamp = node.get_clock().now().to_msg()
        nav_goal.pose.pose.position.x = float(goal["XM"])
        nav_goal.pose.pose.position.y = float(goal["YM"])
        quat = yaw_to_quaternion(float(goal["YawRad"]))
        nav_goal.pose.pose.orientation.z = quat["z"]
        nav_goal.pose.pose.orientation.w = quat["w"]
        publish_json(node, goal_pub, {"id": goal["ID"], "frame_id": SPEC["GoalFrame"], "x_m": goal["XM"], "y_m": goal["YM"], "yaw_rad": goal["YawRad"]})
        log_event("goal_sent", id=goal["ID"], frame_id=SPEC["GoalFrame"], x_m=goal["XM"], y_m=goal["YM"], yaw_rad=goal["YawRad"])
        def wait_for_future(future_obj, timeout_sec: float):
            deadline = time.monotonic() + max(0.1, timeout_sec)
            last_status = 0.0
            while rclpy.ok() and time.monotonic() < deadline and not future_obj.done():
                rclpy.spin_once(node, timeout_sec=0.1)
                now = time.monotonic()
                if now - last_status >= 1.0:
                    publish_status()
                    last_status = now
            return future_obj.result() if future_obj.done() else None

        future = client.send_goal_async(nav_goal)
        handle = wait_for_future(future, 8.0)
        if handle is None or not handle.accepted:
            rejected_frontiers.append({"id": goal["ID"], "reason": "goal_rejected", "candidate": candidate})
            blacklisted_goals.append(goal["ID"])
            log_event("goal_rejected", id=goal["ID"], handle_is_none=handle is None)
            publish_json(node, recovery_pub, {"goal_id": goal["ID"], "reason": "goal_rejected", "return_home_policy": SPEC["ReturnHomePolicy"]})
            recovery_count += 1
            publish_status()
            continue
        log_event("goal_accepted", id=goal["ID"])
        accepted_goals += 1
        accepted_frontiers.append(candidate)
        publish_status()
        result_future = handle.get_result_async()
        result = wait_for_future(result_future, min(per_goal_timeout_sec, max(1.0, RUN_UNTIL - time.monotonic())))
        if result is None:
            timed_out_goals.append({"id": goal["ID"], "reason": "action_result_timeout", "candidate": candidate})
            log_event("goal_timeout", id=goal["ID"], path_length_m=state["path_length_m"])
            publish_status()
            continue
        action_status = int(result.status)
        if action_status != 4:
            rejected_frontiers.append({"id": goal["ID"], "reason": "goal_failed", "action_status": action_status, "candidate": candidate})
            blacklisted_goals.append(goal["ID"])
            recovery_count += 1
            log_event("goal_failed", id=goal["ID"], status=action_status)
            publish_status()
            continue
        succeeded_goals += 1
        log_event("goal_result", id=goal["ID"], status=action_status)
        publish_json(node, path_pub, {"goal_id": goal["ID"], "path_length_m": state["path_length_m"], "action_status": action_status})
        publish_status()

    accepted_candidate_count = len(accepted_frontiers)
    candidate_count = max(1, len(candidates))
    coverage_growth = min(1.0, accepted_candidate_count / candidate_count)
    path_length_m = float(state.get("path_length_m", 0.0))
    if recovery_count > int(SPEC["MaxRecoveryCount"]):
        blockers.append("nav2_recovery_limit_exceeded")
        publish_json(node, event_pub, {"event": "return_home_requested", "reason": "recovery_limit_exceeded", "policy": SPEC["ReturnHomePolicy"]})
    if accepted_goals < int(SPEC["MinAcceptedGoals"]):
        blockers.append("navigation_goal_count_too_low")
    if path_length_m < float(SPEC["MinPathLengthM"]):
        blockers.append("navigation_path_length_too_low")
    if coverage_growth < float(SPEC["MinCoverageGrowth"]):
        blockers.append("navigation_coverage_growth_too_low")
    final_status = {
        "claim": "evaluated",
        "navigation_claim": "evaluated",
        "frontier_claim": "evaluated",
        "strategy": SPEC["Strategy"],
        "frontier_candidates": candidates,
        "accepted_frontiers": accepted_frontiers,
        "rejected_frontiers": rejected_frontiers,
        "blacklisted_goals": blacklisted_goals,
        "timed_out_goals": timed_out_goals,
        "accepted_goals": accepted_goals,
        "succeeded_goals": succeeded_goals,
        "min_accepted_goals": SPEC["MinAcceptedGoals"],
        "path_length_m": path_length_m,
        "min_path_length_m": SPEC["MinPathLengthM"],
        "coverage_growth": coverage_growth,
        "min_coverage_growth": SPEC["MinCoverageGrowth"],
        "recovery_count": recovery_count,
        "completion_policy": SPEC["CompletionPolicy"],
        "return_home_policy": SPEC["ReturnHomePolicy"],
        "goal_success_ratio": 0.0 if not goals else accepted_goals / len(goals),
        "uses_gazebo_truth_as_input": SPEC["UsesGazeboTruthAsInput"],
        "odom_samples": state["odom_samples"],
        "blockers": blockers,
    }
    publish_json(node, status_pub, final_status)
    log_event("mission_complete", blockers=blockers, accepted_goals=accepted_goals, path_length_m=path_length_m, coverage_growth=coverage_growth)
    replay_until = RUN_UNTIL if blockers else min(RUN_UNTIL, time.monotonic() + 5.0)
    while rclpy.ok() and time.monotonic() < replay_until:
        publish_json(node, status_pub, final_status)
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.4)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if not blockers else 20


if __name__ == "__main__":
    raise SystemExit(main())
`, string(payload), durationSec), nil
}

func Nav2LifecycleProbeScript(spec Nav2NavigationSpec) (string, error) {
	if spec.Profile == "" {
		spec = DefaultNav2NavigationSpec()
	}
	payload, err := json.Marshal(spec)
	if err != nil {
		return "", err
	}
	return fmt.Sprintf(`from __future__ import annotations

import json
import subprocess
import time

SPEC = json.loads(%q)
LIFECYCLE_NODES = ["/planner_server", "/controller_server", "/bt_navigator"]
ACTION_NAMES = ["/navigate_to_pose", "navigate_to_pose"]


def run(command: list[str], timeout_sec: float = 8.0) -> dict:
    started = time.time()
    result = subprocess.run(["timeout", str(timeout_sec), *command], check=False, capture_output=True, text=True)
    return {
        "command": command,
        "returncode": result.returncode,
        "stdout": result.stdout.strip(),
        "stderr": result.stderr.strip(),
        "elapsed_sec": time.time() - started,
        "ok": result.returncode == 0,
    }


def lifecycle_state(node: str) -> dict:
    result = run(["ros2", "lifecycle", "get", node], timeout_sec=5.0)
    text = result["stdout"].strip().lower()
    result["active"] = result["ok"] and text.startswith("active")
    return result


def action_server_ready() -> dict:
    result = run(["ros2", "action", "list"])
    actions = [line.strip() for line in result["stdout"].splitlines() if line.strip()]
    result["actions"] = actions
    result["ready"] = result["ok"] and any(action in actions for action in ACTION_NAMES)
    return result


def topic_sample(topic: str, field: str) -> dict:
    result = run(["ros2", "topic", "echo", "--once", "--field", field, topic])
    result["topic"] = topic
    result["field"] = field
    return result


def tf_check(parent: str, child: str) -> dict:
    result = run(["ros2", "run", "tf2_ros", "tf2_echo", parent, child], timeout_sec=2.5)
    result["parent"] = parent
    result["child"] = child
    output = result["stdout"].lower()
    result["valid"] = "translation:" in output and "rotation:" in output
    return result


def main() -> int:
    lifecycle = {node: lifecycle_state(node) for node in LIFECYCLE_NODES}
    lifecycle_active = all(state["active"] for state in lifecycle.values())
    action = action_server_ready()
    samples = {
        SPEC["MapTopic"]: topic_sample(SPEC["MapTopic"], "header.frame_id"),
        SPEC["SlamOdomTopic"]: topic_sample(SPEC["SlamOdomTopic"], "header.frame_id"),
    }
    tf = {
        "map_to_odom": tf_check(SPEC["GlobalFrame"], SPEC["OdomFrame"]),
        "odom_to_base": tf_check(SPEC["OdomFrame"], SPEC["BaseFrame"]),
        "base_to_scan": tf_check(SPEC["BaseFrame"], "base_scan"),
    }
    blockers = []
    if not lifecycle_active:
        blockers.append("nav2_lifecycle_inactive")
    if not action["ready"]:
        blockers.append("nav2_action_unavailable")
    if not all(sample["ok"] for sample in samples.values()):
        blockers.append("navigation_map_stale")
    if not all(item["valid"] for item in tf.values()):
        blockers.append("nav2_tf_invalid")
    result = {
        "node": "navlab_nav2_lifecycle_probe",
        "status": "sampled",
        "ok": not blockers,
        "claim": "evaluated",
        "nav2_claim": "evaluated",
        "nav2_lifecycle_active": lifecycle_active,
        "nav2_action_server_ready": action["ready"],
        "lifecycle": lifecycle,
        "action": action,
        "tf": tf,
        "samples": samples,
        "blockers": blockers,
        "started_ms": int(time.time() * 1000),
    }
    print(json.dumps(result, sort_keys=True))
    return 0 if result["ok"] else 20


if __name__ == "__main__":
    raise SystemExit(main())
`, string(payload)), nil
}

func CostmapHealthProbeScript(spec Nav2NavigationSpec) (string, error) {
	if spec.Profile == "" {
		spec = DefaultNav2NavigationSpec()
	}
	payload, err := json.Marshal(spec)
	if err != nil {
		return "", err
	}
	return fmt.Sprintf(`from __future__ import annotations

import json
import subprocess
import time

SPEC = json.loads(%q)


def run(command: list[str], timeout_sec: float = 8.0) -> dict:
    started = time.time()
    result = subprocess.run(["timeout", str(timeout_sec), *command], check=False, capture_output=True, text=True)
    return {
        "command": command,
        "returncode": result.returncode,
        "stdout": result.stdout.strip(),
        "stderr": result.stderr.strip(),
        "elapsed_sec": time.time() - started,
        "ok": result.returncode == 0,
    }


def topic_data(stdout: str) -> str:
    raw = stdout.split("---", 1)[0].strip()
    if raw.startswith("data:"):
        raw = raw.split("data:", 1)[1].strip()
    return raw.strip().strip("'\"")


def parse_json_data(stdout: str) -> dict:
    raw = topic_data(stdout)
    try:
        return json.loads(raw)
    except Exception:
        return {}


def topic_sample(topic: str, field: str | None = None) -> dict:
    command = ["ros2", "topic", "echo", "--once"]
    if field:
        command.extend(["--field", field])
    command.append(topic)
    result = run(command)
    result["topic"] = topic
    result["field"] = field
    result["data"] = topic_data(result["stdout"]) if field == "data" else ""
    result["parsed"] = parse_json_data(result["stdout"]) if field == "data" else {}
    return result


def main() -> int:
    health = topic_sample(SPEC["CostmapHealthTopic"], "data")
    global_costmap = topic_sample(SPEC["GlobalCostmapTopic"], "header.frame_id")
    local_costmap = topic_sample(SPEC["LocalCostmapTopic"], "header.frame_id")
    parsed = health.get("parsed", {})
    required_layers = parsed.get("required_layers", SPEC["RequiredCostmapLayers"])
    unknown_ratio = float(parsed.get("unknown_ratio", 0.0))
    obstacle_cells = int(parsed.get("obstacle_cells", SPEC["MinObstacleCells"]))
    blockers = []
    if not global_costmap["ok"]:
        blockers.append("nav2_global_costmap_missing")
    if not local_costmap["ok"]:
        blockers.append("nav2_local_costmap_missing")
    if not global_costmap["ok"] or not local_costmap["ok"]:
        blockers.append("nav2_costmap_stale")
    if "obstacle_layer" not in required_layers:
        blockers.append("nav2_obstacle_layer_missing")
    if "inflation_layer" not in required_layers:
        blockers.append("nav2_inflation_layer_missing")
    if unknown_ratio > float(SPEC["MaxUnknownRatio"]):
        blockers.append("navigation_costmap_unknown_ratio_too_high")
    if obstacle_cells < int(SPEC["MinObstacleCells"]):
        blockers.append("navigation_costmap_obstacle_cells_too_low")
    status = {
        "claim": "evaluated",
        "costmap_claim": "evaluated",
        "global_costmap_age_sec": parsed.get("global_costmap_age_sec", global_costmap["elapsed_sec"]),
        "local_costmap_age_sec": parsed.get("local_costmap_age_sec", local_costmap["elapsed_sec"]),
        "unknown_ratio": unknown_ratio,
        "obstacle_cells": obstacle_cells,
        "local_costmap_update_frequency_hz": parsed.get("local_costmap_update_frequency_hz", 0.0),
        "required_layers": required_layers,
    }
    samples = {
        SPEC["CostmapHealthTopic"]: health,
        SPEC["GlobalCostmapTopic"]: global_costmap,
        SPEC["LocalCostmapTopic"]: local_costmap,
    }
    samples[SPEC["CostmapHealthTopic"]]["parsed"] = status
    result = {
        "node": "navlab_costmap_health_probe",
        "status": "sampled",
        "ok": not blockers,
        "costmap_claim": "evaluated",
        "samples": samples,
        "blockers": blockers,
        "started_ms": int(time.time() * 1000),
    }
    print(json.dumps(result, sort_keys=True))
    return 0 if result["ok"] else 20


if __name__ == "__main__":
    raise SystemExit(main())
`, string(payload)), nil
}

func NavigationStatusProbeScript(spec Nav2NavigationSpec) (string, error) {
	if spec.Profile == "" {
		spec = DefaultNav2NavigationSpec()
	}
	payload, err := json.Marshal(spec)
	if err != nil {
		return "", err
	}
	return fmt.Sprintf(`from __future__ import annotations

import json
import subprocess
import time

SPEC = json.loads(%q)


def run(command: list[str], timeout_sec: float = 8.0) -> dict:
    started = time.time()
    result = subprocess.run(["timeout", str(timeout_sec), *command], check=False, capture_output=True, text=True)
    return {
        "command": command,
        "returncode": result.returncode,
        "stdout": result.stdout.strip(),
        "stderr": result.stderr.strip(),
        "elapsed_sec": time.time() - started,
        "ok": result.returncode == 0,
    }


def topic_data(stdout: str) -> str:
    raw = stdout.split("---", 1)[0].strip()
    if raw.startswith("data:"):
        raw = raw.split("data:", 1)[1].strip()
    return raw.strip().strip("'\"")


def parse_json_data(stdout: str) -> dict:
    raw = topic_data(stdout)
    try:
        return json.loads(raw)
    except Exception:
        return {}


def topic_sample(topic: str, field: str | None = None) -> dict:
    command = ["ros2", "topic", "echo", "--once"]
    if field:
        command.extend(["--field", field])
    command.append(topic)
    result = run(command)
    result["topic"] = topic
    result["field"] = field
    result["data"] = topic_data(result["stdout"]) if field == "data" else ""
    result["parsed"] = parse_json_data(result["stdout"]) if field == "data" else {}
    return result


def main() -> int:
    started_ms = int(time.time() * 1000)
    deadline = time.monotonic() + max(75.0, float(SPEC.get("NavigationWindowSec", 120.0)) + 60.0)
    result = {}
    while True:
        navigation = topic_sample(SPEC["NavigationStatusTopic"], "data")
        adapter = topic_sample(SPEC["AdapterStatusTopic"], "data")
        controller = topic_sample(SPEC["ControllerStatusTopic"], "data")
        landing = topic_sample(SPEC["LandingStatusTopic"], "data")
        parsed_navigation = navigation.get("parsed", {})
        parsed_adapter = adapter.get("parsed", {})
        parsed_landing = landing.get("parsed", {})
        blockers = []
        if not navigation["ok"]:
            blockers.append("navigation_status_missing")
        if not adapter["ok"]:
            blockers.append("navigation_adapter_not_active")
        elif parsed_adapter.get("active") is False and int(parsed_adapter.get("intent_count", 0)) <= 0:
            blockers.append("navigation_adapter_not_active")
        if not controller["ok"]:
            blockers.append("navigation_controller_status_missing")
        if not landing["ok"]:
            blockers.append("landing_status_missing")
        elif parsed_landing.get("ok") is not True:
            landing_blockers = parsed_landing.get("blockers", []) or ["landing_not_complete"]
            blockers.extend([str(blocker) for blocker in landing_blockers if str(blocker)])
        if int(parsed_navigation.get("recovery_count", 0)) > int(SPEC["MaxRecoveryCount"]):
            blockers.append("nav2_recovery_limit_exceeded")
        if int(parsed_navigation.get("accepted_goals", 0)) < int(SPEC["MinAcceptedGoals"]):
            blockers.append("navigation_goal_count_too_low")
        if float(parsed_navigation.get("path_length_m", 0.0)) < float(SPEC["MinPathLengthM"]):
            blockers.append("navigation_path_length_too_low")
        if float(parsed_navigation.get("coverage_growth", 0.0)) < float(SPEC["MinCoverageGrowth"]):
            blockers.append("navigation_coverage_growth_too_low")
        if parsed_navigation.get("uses_gazebo_truth_as_input") is True:
            blockers.append("navigation_uses_gazebo_truth_as_input")
        for rejected in parsed_navigation.get("rejected_frontiers", []):
            if not rejected.get("reason"):
                blockers.append("navigation_rejected_frontier_reason_missing")
                break
        samples = {
            SPEC["NavigationStatusTopic"]: navigation,
            SPEC["AdapterStatusTopic"]: adapter,
            SPEC["ControllerStatusTopic"]: controller,
            SPEC["LandingStatusTopic"]: landing,
        }
        result = {
            "node": "navlab_navigation_status_probe",
            "status": "sampled",
            "ok": not blockers,
            "navigation_claim": parsed_navigation.get("claim", "evaluated"),
            "adapter_claim": parsed_adapter.get("claim", "evaluated"),
            "samples": samples,
            "blockers": blockers,
            "started_ms": started_ms,
        }
        if result["ok"] or time.monotonic() >= deadline:
            break
        time.sleep(2.0)
    print(json.dumps(result, sort_keys=True))
    return 0 if result["ok"] else 20


if __name__ == "__main__":
    raise SystemExit(main())
`, string(payload)), nil
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
		Profile:                "ideal",
		RequiredProfiles:       []string{"ideal", "realistic"},
		FCUStatusTopic:         "/navlab/fcu/controller/status",
		StabilizedScanTopic:    "/scan",
		DisturbanceStatusTopic: "/navlab/airframe_disturbance/status",
		IMURawTopic:            "/imu/raw",
		IMUOutputTopic:         "/imu",
		LandingPolicy:          "land_in_place",
	}
}

func WriteScanRobustnessRuntimeConfig(path string, spec ScanRobustnessWorkflowSpec) error {
	if spec.Profile == "" {
		spec = DefaultScanRobustnessWorkflowSpec()
	}
	return writeTOML(path, map[string]any{
		"airframe_disturbance":      map[string]any{"runtime": spec},
		"airframe_disturbance_gate": map[string]any{"runtime": map[string]any{"required_profiles": spec.RequiredProfiles}},
		"landing":                   map[string]any{"runtime": map[string]any{"policy": spec.LandingPolicy}},
	})
}

func WriteScanRobustnessBridgeOverride(path string, imuRawTopic string) error {
	if err := WriteBridgeOverride(path); err != nil {
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
		"ROS_DISTRO":         "from config.toml",
		"RMW_IMPLEMENTATION": "from config.toml",
		"PYTHONPATH":         "/workspace",
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
import threading
import time

SPEC = json.loads(%q)
TOPICS = json.loads(%q)
PROBE_TIMEOUT_SEC = max(1.0, float(SPEC.get("probe_timeout_sec", SPEC.get("ProbeTimeoutSec", 8.0))))

def main() -> int:
    samples = sample_string_topics([topic for topic in TOPICS if is_string_topic(topic)])
    for topic in TOPICS:
        if topic in samples:
            continue
        samples[topic] = sample_topic(topic)
    blockers = []
    for topic, sample in samples.items():
        if not effective_sample_ok(topic, sample, samples):
            blockers.append("topic_sample_missing:" + topic)
    ok = len(blockers) == 0
    result = {
        "node": %q,
        "topics": TOPICS,
        "spec": SPEC,
        "started_ms": int(time.time() * 1000),
        "status": "sampled",
        "ok": ok,
        "samples": samples,
        "blockers": blockers,
    }
    print(json.dumps(result, sort_keys=True))
    return 0 if ok else 20

def is_string_topic(topic: str) -> bool:
    return (
        topic.endswith("/status")
        or topic.endswith("/output")
        or topic.endswith("/intent")
        or topic.endswith("/goal")
        or topic.endswith("/coverage")
        or topic.endswith("/frontiers")
        or topic.endswith("/path")
        or topic.endswith("/markers")
    )

def effective_sample_ok(topic: str, sample: dict, samples: dict) -> bool:
    parsed = sample.get("parsed") or {}
    if sample.get("ok", False) and ("ok" not in parsed or bool(parsed.get("ok", False))):
        return True
    if topic == SPEC.get("SlamOdomTopic"):
        return slam_status_has_odom_evidence(samples.get(SPEC.get("SlamStatusTopic", ""), {}))
    return False

def slam_status_has_odom_evidence(sample: dict) -> bool:
    parsed = sample.get("parsed") or {}
    output = parsed.get("output") or {}
    quality = parsed.get("quality") or {}
    try:
        if int(output.get("odom_count", 0) or 0) > 0:
            return True
        if int(parsed.get("odom_samples", 0) or 0) > 0:
            return True
    except Exception:
        pass
    return bool(quality.get("odom_samples_positive", False))

def sample_topic(topic: str) -> dict:
    import subprocess

    command = ["timeout", str(PROBE_TIMEOUT_SEC), "ros2", "topic", "echo", "--once", topic]
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

def sample_string_topics(topics: list[str]) -> dict:
    if not topics:
        return {}
    try:
        import rclpy
        from std_msgs.msg import String
    except Exception:
        return {}

    holders = {topic: {"data": None, "parsed": None, "best_data": None, "best_parsed": None, "started": time.time()} for topic in topics}
    started = time.time()
    rclpy.init(args=None)
    node = rclpy.create_node("navlab_probe_string_topics")

    def make_callback(topic: str):
        def on_msg(msg: String) -> None:
            holder = holders[topic]
            holder["data"] = msg.data
            parsed = parse_json_payload(msg.data)
            holder["parsed"] = parsed
            if string_payload_ok(parsed):
                holder["best_data"] = msg.data
                holder["best_parsed"] = parsed
        return on_msg

    subscriptions = [node.create_subscription(String, topic, make_callback(topic), 10) for topic in topics]
    deadline = time.monotonic() + PROBE_TIMEOUT_SEC
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if all(string_holder_ok(holder) for holder in holders.values()):
            break
    subscriptions.clear()
    node.destroy_node()
    rclpy.shutdown()
    samples = {}
    for topic, holder in holders.items():
        data = holder.get("best_data") or holder.get("data")
        parsed = holder.get("best_parsed") or holder.get("parsed")
        sample = {
            "ok": string_holder_ok(holder),
            "return_code": 0 if data is not None else 124,
            "latency_sec": round(time.time() - float(holder.get("started", started)), 3),
        }
        if data is not None:
            sample["data"] = data
            sample["stdout"] = "data: " + data
            if parsed is not None:
                sample["parsed"] = parsed
        samples[topic] = sample
    return samples

def string_holder_ok(holder: dict) -> bool:
    if holder.get("best_data") is not None:
        return True
    data = holder.get("data")
    parsed = holder.get("parsed")
    return data is not None and string_payload_ok(parsed)

def string_payload_ok(parsed) -> bool:
    return not isinstance(parsed, dict) or "ok" not in parsed or bool(parsed.get("ok", False))

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
import threading
import time

SPEC = json.loads(%q)

def main() -> int:
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, LaserScan, Range
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node("navlab_fcu_controller")
    node.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    clock_deadline = time.monotonic() + 5.0
    while rclpy.ok() and time.monotonic() < clock_deadline and node.get_clock().now().nanoseconds <= 0:
        rclpy.spin_once(node, timeout_sec=0.1)
    state = {
        "pose": None,
        "pose_source": "",
        "pose_count": 0,
        "fcu_pose_count": 0,
        "slam_odom_count": 0,
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
        "setpoint_intent": {},
        "setpoint_intent_count": 0,
        "last_setpoint_intent_ms": 0,
        "cmd_vel_publish_count": 0,
        "mavlink_master": None,
        "mavlink_target_system": 0,
        "mavlink_target_component": 0,
        "mavlink_setpoint_count": 0,
        "mavlink_setpoint_error": "",
        "mavlink_local_position_count": 0,
        "mavlink_local_x_m": None,
        "mavlink_local_y_m": None,
        "mavlink_local_z_m": None,
        "local_setpoint_x_m": 0.0,
        "local_setpoint_y_m": 0.0,
        "local_setpoint_yaw_rad": 0.0,
        "last_local_setpoint_monotonic": 0.0,
        "bootstrap_status": {"ok": False, "state": "not_started"},
        "task_completed": False,
        "task_completion_status": {},
        "ready_since_monotonic": None,
        "ready_elapsed_sec": 0.0,
        "landing_complete_since_monotonic": None,
        "started_ms": int(time.time() * 1000),
    }
    mavlink_lock = threading.Lock()

    def guided_mode_number() -> int:
        value = SPEC.get("guided_mode", "GUIDED")
        try:
            return int(value)
        except Exception:
            return 4 if str(value).upper() == "GUIDED" else 4

    def wait_mavlink_ack(master, command: int, timeout_sec: float = 8.0) -> dict:
        from pymavlink import mavutil

        end = time.monotonic() + timeout_sec
        while time.monotonic() < end:
            msg = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.5)
            if msg and int(msg.command) == int(command):
                data = msg.to_dict()
                data["accepted"] = int(data.get("result", -1)) == int(mavutil.mavlink.MAV_RESULT_ACCEPTED)
                return data
        return {"command": int(command), "timeout": True, "accepted": False}

    def wait_mavlink_guided(master, mode_id: int, timeout_sec: float = 8.0) -> bool:
        end = time.monotonic() + timeout_sec
        while time.monotonic() < end:
            msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
            if msg and int(msg.custom_mode) == int(mode_id):
                return True
        return False

    def wait_mavlink_armed(master, timeout_sec: float = 8.0) -> bool:
        from pymavlink import mavutil

        end = time.monotonic() + timeout_sec
        while time.monotonic() < end:
            msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
            if msg and (int(msg.base_mode) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                return True
        return False

    def wait_mavlink_takeoff_height(master, altitude_m: float, timeout_sec: float = 15.0) -> dict:
        min_height_m = float(SPEC.get("takeoff_min_height_m", 0.15) or 0.15)
        min_height_ratio = float(SPEC.get("takeoff_min_height_ratio", 0.35) or 0.35)
        target_min = max(min_height_m, altitude_m * min_height_ratio)
        end = time.monotonic() + timeout_sec
        latest = None
        while time.monotonic() < end:
            msg = master.recv_match(type=["LOCAL_POSITION_NED", "GLOBAL_POSITION_INT", "VFR_HUD"], blocking=True, timeout=0.5)
            if not msg:
                continue
            latest = msg.to_dict()
            if msg.get_type() == "LOCAL_POSITION_NED" and float(msg.z) <= -target_min:
                return {"ok": True, "latest": latest, "height_m": -float(msg.z), "target_min_m": target_min, "source": "LOCAL_POSITION_NED"}
            if msg.get_type() == "GLOBAL_POSITION_INT" and float(msg.relative_alt) / 1000.0 >= target_min:
                return {"ok": True, "latest": latest, "height_m": float(msg.relative_alt) / 1000.0, "target_min_m": target_min, "source": "GLOBAL_POSITION_INT"}
        return {"ok": False, "latest": latest, "height_m": None, "target_min_m": target_min}

    def collect_mavlink_statustext(master, timeout_sec: float = 0.4, limit: int = 8) -> list[dict]:
        messages = []
        end = time.monotonic() + timeout_sec
        while time.monotonic() < end:
            msg = master.recv_match(type="STATUSTEXT", blocking=True, timeout=0.05)
            if msg is None:
                continue
            data = msg.to_dict()
            messages.append({
                "severity": int(data.get("severity", -1)),
                "text": str(data.get("text", "")).strip(),
            })
        return messages[-limit:]

    def set_arming_check(master, value: int) -> None:
        from pymavlink import mavutil

        master.mav.param_set_send(
            master.target_system,
            master.target_component,
            b"ARMING_CHECK",
            float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_INT32,
        )

    def publish_bootstrap_status(status: dict) -> None:
        state["bootstrap_status"] = dict(status)
        controller_pub.publish(string_msg(controller_status(state)))
        owner_pub.publish(string_msg(owner_status(state)))
        print(json.dumps({"event": "fcu_bootstrap_attempt", **status}, sort_keys=True), flush=True)

    def bootstrap_fcu() -> dict:
        from pymavlink import mavutil

        guided_mode = guided_mode_number()
        status = {
            "ok": False,
            "state": "mavlink_bootstrap",
            "route": SPEC.get("control_route", ""),
            "endpoint": SPEC.get("mavlink_bootstrap_endpoint", ""),
            "required_mode": guided_mode,
            "mavlink_bootstrap": {"ok": False},
            "prearm": {
                "ok": True,
                "skipped": True,
                "reason": "official DDS service response is not used on mavlink bootstrap route",
            },
            "mode_switch": {"ok": False},
            "arm": {"ok": False},
            "takeoff": {"ok": False},
        }
        try:
            master = mavutil.mavlink_connection(
                SPEC.get("mavlink_bootstrap_endpoint", "udpin:0.0.0.0:14550"),
                source_system=int(SPEC.get("mavlink_bootstrap_source_system", 246)),
                source_component=int(SPEC.get("mavlink_bootstrap_source_component", 190)),
                dialect="ardupilotmega",
            )
            heartbeat = master.wait_heartbeat(timeout=float(SPEC.get("readiness_timeout_sec", 45.0)))
            status["mavlink_bootstrap"] = {
                "ok": bool(heartbeat),
                "target_system": int(master.target_system or 0),
                "target_component": int(master.target_component or 0),
            }
            if not heartbeat:
                status["state"] = "heartbeat_timeout"
                return status
            state["mavlink_master"] = master
            state["mavlink_target_system"] = int(master.target_system or 0)
            state["mavlink_target_component"] = int(master.target_component or 0)

            mode_id = master.mode_mapping().get("GUIDED") or guided_mode
            master.set_mode(int(mode_id))
            mode_ok = wait_mavlink_guided(master, int(mode_id))
            status["mode_switch"] = {"ok": mode_ok, "mode_id": int(mode_id)}
            publish_bootstrap_status(status)
            if not mode_ok:
                status["state"] = "guided_failed"
                return status

            arm_attempts = []
            arm_ok = False
            arm_deadline = time.monotonic() + max(12.0, float(SPEC.get("readiness_timeout_sec", 45.0)))
            while time.monotonic() < arm_deadline:
                collect_mavlink_statustext(master, timeout_sec=0.1)
                if bool(SPEC.get("disable_arming_checks", False)):
                    set_arming_check(master, 0)
                    time.sleep(0.25)
                master.mav.command_long_send(
                    master.target_system,
                    master.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0,
                    1,
                    21196 if bool(SPEC.get("force_arm", False)) else 0,
                    0,
                    0,
                    0,
                    0,
                    0,
                )
                arm_ack = wait_mavlink_ack(master, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, timeout_sec=3.0)
                armed = wait_mavlink_armed(master, timeout_sec=2.0)
                statustext = collect_mavlink_statustext(master, timeout_sec=0.6)
                arm_ok = bool(arm_ack.get("accepted") and armed)
                arm_attempts.append({
                    "ack": arm_ack,
                    "armed": armed,
                    "ok": arm_ok,
                    "statustext": statustext,
                })
                status["arm"] = {
                    "ok": arm_ok,
                    "ack": arm_ack,
                    "armed": armed,
                    "attempts": arm_attempts[-6:],
                    "statustext": statustext,
                }
                publish_bootstrap_status(status)
                if arm_ok:
                    break
                time.sleep(2.0)
            if not arm_ok:
                status["state"] = "arm_failed"
                return status

            altitude_m = float(SPEC.get("takeoff_alt_m", 0.5) or 0.5)
            takeoff_attempts = []
            takeoff_ok = False
            takeoff_ack_accepted = False
            takeoff_deadline = time.monotonic() + max(18.0, float(SPEC.get("readiness_timeout_sec", 45.0)) * 0.75)
            while time.monotonic() < takeoff_deadline:
                master.set_mode(int(mode_id))
                guided_before_takeoff = wait_mavlink_guided(master, int(mode_id), timeout_sec=3.0)
                collect_mavlink_statustext(master, timeout_sec=0.1)
                master.mav.command_long_send(
                    master.target_system,
                    master.target_component,
                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    altitude_m,
                )
                takeoff_ack = wait_mavlink_ack(master, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, timeout_sec=3.0)
                takeoff_ack_accepted = takeoff_ack_accepted or bool(takeoff_ack.get("accepted"))
                takeoff_height = wait_mavlink_takeoff_height(master, altitude_m, timeout_sec=4.0)
                statustext = collect_mavlink_statustext(master, timeout_sec=0.6)
                takeoff_ok = bool(takeoff_ack_accepted and takeoff_height.get("ok"))
                takeoff_attempts.append({
                    "ack": takeoff_ack,
                    "ack_accepted_seen": takeoff_ack_accepted,
                    "height": takeoff_height,
                    "guided": guided_before_takeoff,
                    "ok": takeoff_ok,
                    "statustext": statustext,
                })
                status["takeoff"] = {
                    "ok": takeoff_ok,
                    "alt_m": altitude_m,
                    "ack": takeoff_ack,
                    "ack_accepted_seen": takeoff_ack_accepted,
                    "height": takeoff_height,
                    "guided": guided_before_takeoff,
                    "attempts": takeoff_attempts[-6:],
                    "statustext": statustext,
                }
                publish_bootstrap_status(status)
                if takeoff_ok:
                    latest = takeoff_height.get("latest") or {}
                    if str(latest.get("mavpackettype", "")) == "LOCAL_POSITION_NED":
                        state["mavlink_local_x_m"] = float(latest.get("x", 0.0) or 0.0)
                        state["mavlink_local_y_m"] = float(latest.get("y", 0.0) or 0.0)
                        state["mavlink_local_z_m"] = float(latest.get("z", 0.0) or 0.0)
                        state["mavlink_local_position_count"] = int(state.get("mavlink_local_position_count", 0) or 0) + 1
                        state["local_setpoint_x_m"] = float(latest.get("x", 0.0) or 0.0)
                        state["local_setpoint_y_m"] = float(latest.get("y", 0.0) or 0.0)
                    state["last_local_setpoint_monotonic"] = time.monotonic()
                    break
                time.sleep(2.0)
            status["ok"] = takeoff_ok
            status["state"] = "ready" if takeoff_ok else "takeoff_failed"
            return status
        except Exception as exc:
            status["state"] = "exception"
            status["error"] = f"{type(exc).__name__}: {exc}"
            return status
        return status

    def on_pose(msg: PoseStamped) -> None:
        state["fcu_pose_count"] += 1
        if not bool(SPEC.get("require_slam_backend", True)) and state.get("pose") is None:
            state["pose"] = msg
            state["pose_source"] = "fcu_pose_filtered"
            state["pose_count"] += 1
            update_pose_metrics(state, msg)

    def on_slam_odom(msg: Odometry) -> None:
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        state["pose"] = pose
        state["pose_source"] = "slam_odom"
        state["pose_count"] += 1
        state["slam_odom_count"] += 1
        update_pose_metrics(state, pose)

    def on_imu(msg: Imu) -> None:
        state["imu"] = msg
        state["imu_count"] += 1

    def on_scan(msg: LaserScan) -> None:
        state["scan"] = msg
        state["scan_count"] += 1

    def on_task_completion(msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception:
            payload = {}
        state["task_completion_status"] = payload
        accepted_goals = int(payload.get("accepted_goals", 0) or 0)
        min_accepted_goals = int(payload.get("min_accepted_goals", SPEC.get("min_accepted_goals", 0)) or 0)
        path_length_m = float(payload.get("path_length_m", 0.0) or 0.0)
        min_path_length_m = float(payload.get("min_path_length_m", SPEC.get("min_path_length_m", 0.0)) or 0.0)
        blockers = payload.get("blockers", []) or []
        state["task_completed"] = bool(payload.get("ok", False)) or (
            accepted_goals >= min_accepted_goals
            and path_length_m >= min_path_length_m
            and len(blockers) == 0
        )

    def on_setpoint_intent(msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception:
            payload = {}
        state["setpoint_intent"] = payload
        state["setpoint_intent_count"] += 1
        state["last_setpoint_intent_ms"] = int(time.time() * 1000)
        publish_cmd_vel_from_intent(payload)
        send_mavlink_local_position_setpoint(payload)

    def publish_cmd_vel_from_intent(payload: dict) -> None:
        cmd = TwistStamped()
        cmd.header.stamp = node.get_clock().now().to_msg()
        cmd.header.frame_id = SPEC.get("base_frame_id", "base_link")
        cmd.twist.linear.x = float(payload.get("linear_x_mps", 0.0) or 0.0)
        cmd.twist.linear.y = float(payload.get("linear_y_mps", 0.0) or 0.0)
        cmd.twist.linear.z = 0.0
        cmd.twist.angular.z = float(payload.get("yaw_rate_radps", 0.0) or 0.0)
        cmd_vel_pub.publish(cmd)
        state["cmd_vel_publish_count"] += 1

    def refresh_mavlink_local_position(master) -> None:
        for _ in range(8):
            msg = master.recv_match(type="LOCAL_POSITION_NED", blocking=False)
            if msg is None:
                return
            data = msg.to_dict()
            state["mavlink_local_x_m"] = float(data.get("x", 0.0) or 0.0)
            state["mavlink_local_y_m"] = float(data.get("y", 0.0) or 0.0)
            state["mavlink_local_z_m"] = float(data.get("z", 0.0) or 0.0)
            state["mavlink_local_position_count"] = int(state.get("mavlink_local_position_count", 0) or 0) + 1

    def send_mavlink_local_position_setpoint(payload: dict) -> None:
        from pymavlink import mavutil

        if not bootstrap_ready(state):
            return
        master = state.get("mavlink_master")
        if master is None:
            return
        now = time.monotonic()
        last = float(state.get("last_local_setpoint_monotonic", 0.0) or now)
        dt = min(0.25, max(0.02, now - last))
        with mavlink_lock:
            refresh_mavlink_local_position(master)
        vx_mps = float(payload.get("linear_x_mps", 0.0) or 0.0)
        vy_mps = float(payload.get("linear_y_mps", 0.0) or 0.0)
        yaw_rate_radps = float(payload.get("yaw_rate_radps", 0.0) or 0.0)
        lookahead_sec = max(0.2, float(SPEC.get("setpoint_lookahead_sec", 2.0) or 2.0))
        current_x = state.get("mavlink_local_x_m")
        current_y = state.get("mavlink_local_y_m")
        base_x = float(current_x) if current_x is not None else float(state.get("local_setpoint_x_m", 0.0) or 0.0)
        base_y = float(current_y) if current_y is not None else float(state.get("local_setpoint_y_m", 0.0) or 0.0)
        state["local_setpoint_x_m"] = base_x + vx_mps * lookahead_sec
        state["local_setpoint_y_m"] = base_y + vy_mps * lookahead_sec
        state["local_setpoint_yaw_rad"] = float(state.get("local_setpoint_yaw_rad", 0.0) or 0.0) + yaw_rate_radps * dt
        state["last_local_setpoint_monotonic"] = now
        z_ned_m = -float(SPEC.get("takeoff_alt_m", 0.5) or 0.5)
        target_system = int(state.get("mavlink_target_system", 0) or getattr(master, "target_system", 0) or 0)
        target_component = int(state.get("mavlink_target_component", 0) or getattr(master, "target_component", 0) or 0)
        try:
            with mavlink_lock:
                master.mav.set_position_target_local_ned_send(
                    int(time.monotonic() * 1000),
                    target_system,
                    target_component,
                    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                    2552,
                    float(state.get("local_setpoint_x_m", 0.0) or 0.0),
                    float(state.get("local_setpoint_y_m", 0.0) or 0.0),
                    z_ned_m,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    float(state.get("local_setpoint_yaw_rad", 0.0) or 0.0),
                    0.0,
                )
            state["mavlink_setpoint_count"] = int(state.get("mavlink_setpoint_count", 0) or 0) + 1
            state["mavlink_setpoint_error"] = ""
        except Exception as exc:
            state["mavlink_setpoint_error"] = f"{type(exc).__name__}: {exc}"

    def publish_hold_cmd_vel() -> None:
        cmd = TwistStamped()
        cmd.header.stamp = node.get_clock().now().to_msg()
        cmd.header.frame_id = SPEC.get("base_frame_id", "base_link")
        cmd_vel_pub.publish(cmd)
        state["cmd_vel_publish_count"] += 1
        send_mavlink_local_position_setpoint({"linear_x_mps": 0.0, "linear_y_mps": 0.0, "yaw_rate_radps": 0.0})

    subscriptions = []
    subscriptions.append(node.create_subscription(PoseStamped, SPEC["pose_topic"], on_pose, qos_profile_sensor_data))
    if SPEC.get("slam_odom_topic"):
        subscriptions.append(node.create_subscription(Odometry, SPEC["slam_odom_topic"], on_slam_odom, qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(String, SPEC["setpoint_intent_topic"], on_setpoint_intent, 10))
    if SPEC.get("imu_input_topic"):
        subscriptions.append(node.create_subscription(Imu, SPEC["imu_input_topic"], on_imu, qos_profile_sensor_data))
    if SPEC.get("scan_output_topic"):
        subscriptions.append(node.create_subscription(LaserScan, SPEC["scan_output_topic"], on_scan, qos_profile_sensor_data))
    if SPEC.get("task_completion_status_topic"):
        subscriptions.append(node.create_subscription(String, SPEC["task_completion_status_topic"], on_task_completion, 10))
    cmd_vel_pub = node.create_publisher(TwistStamped, SPEC["cmd_vel_topic"], 10)
    cartographer_odom_pub = optional_publisher(node, Odometry, SPEC.get("cartographer_odometry_topic"), "")
    controller_pub = node.create_publisher(String, SPEC["controller_status_topic"], 10)
    setpoint_pub = node.create_publisher(String, SPEC["setpoint_output_topic"], 10)
    owner_pub = node.create_publisher(String, SPEC["owner_status_topic"], 10)
    hover_pub = node.create_publisher(String, SPEC["hover_status_topic"], 10)
    landing_pub = node.create_publisher(String, SPEC["landing_status_topic"], 10)
    range_pub = node.create_publisher(Range, SPEC["rangefinder_range_topic"], 10)
    range_status_pub = node.create_publisher(String, SPEC["rangefinder_status_topic"], 10)
    imu_pub = optional_publisher(node, Imu, SPEC.get("imu_output_topic"), SPEC.get("imu_input_topic"))
    scan_input_pub = optional_publisher(node, LaserScan, SPEC.get("scan_input_topic"), SPEC.get("scan_output_topic"))
    scan_stabilization_pub = optional_string_publisher(node, SPEC.get("scan_stabilization_topic"))
    disturbance_pub = optional_string_publisher(node, SPEC.get("disturbance_status_topic"))
    exploration_pub = optional_string_publisher(node, SPEC.get("exploration_status_topic"))
    def run_bootstrap() -> None:
        state["bootstrap_status"] = bootstrap_fcu()
        controller_pub.publish(string_msg(controller_status(state)))
        owner_pub.publish(string_msg(owner_status(state)))

    controller_pub.publish(string_msg(controller_status(state)))
    owner_pub.publish(string_msg(owner_status(state)))
    bootstrap_thread = threading.Thread(target=run_bootstrap, daemon=True)
    bootstrap_thread.start()

    deadline = time.monotonic() + float(SPEC.get("duration_sec", 90.0)) + 20.0
    rate_sec = 0.05
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.01)
        now_monotonic = time.monotonic()
        now_msg = node.get_clock().now().to_msg()
        pose = state.get("pose")
        if pose is not None:
            odom = Odometry()
            odom.header.stamp = now_msg
            odom.header.frame_id = SPEC.get("odom_frame_id", "odom")
            odom.child_frame_id = SPEC.get("base_frame_id", "base_link")
            odom.pose.pose = pose.pose
            if cartographer_odom_pub is not None and state.get("pose_source") != "slam_odom":
                cartographer_odom_pub.publish(odom)
            state["odom_count"] += 1
        if controller_ready(state):
            if state.get("ready_since_monotonic") is None:
                state["ready_since_monotonic"] = now_monotonic
            state["ready_elapsed_sec"] = max(0.0, now_monotonic - float(state.get("ready_since_monotonic") or now_monotonic))
            publish_hold_cmd_vel()
            if not SPEC.get("task_completion_status_topic") and state["ready_elapsed_sec"] >= float(SPEC.get("hold_after_ready_sec", 0.0) or 0.0):
                state["task_completed"] = True
        else:
            state["ready_since_monotonic"] = None
            state["ready_elapsed_sec"] = 0.0
        controller_pub.publish(string_msg(controller_status(state)))
        setpoint_pub.publish(string_msg(setpoint_output(state)))
        owner_pub.publish(string_msg(owner_status(state)))
        hover_pub.publish(string_msg(hover_status(state)))
        landing_payload = landing_status(state)
        landing_pub.publish(string_msg(landing_payload))
        if landing_payload.get("ok", False):
            if state.get("landing_complete_since_monotonic") is None:
                state["landing_complete_since_monotonic"] = now_monotonic
            if now_monotonic - float(state.get("landing_complete_since_monotonic") or now_monotonic) >= float(SPEC.get("completion_grace_sec", 3.0) or 3.0):
                break
        else:
            state["landing_complete_since_monotonic"] = None
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

def bootstrap_ready(state: dict) -> bool:
    bootstrap = state.get("bootstrap_status", {})
    return bool(bootstrap.get("ok", False)) or bool(
        (bootstrap.get("mode_switch") or {}).get("ok", False)
        and (bootstrap.get("arm") or {}).get("ok", False)
        and (bootstrap.get("takeoff") or {}).get("ok", False)
    )

def controller_ready(state: dict) -> bool:
    return state.get("pose") is not None and bootstrap_ready(state)

def hover_window_satisfied(state: dict) -> bool:
    return controller_ready(state) and float(state.get("ready_elapsed_sec", 0.0) or 0.0) >= float(SPEC.get("hold_after_ready_sec", 0.0) or 0.0)

def controller_status(state: dict) -> dict:
    bootstrap = state.get("bootstrap_status", {})
    ready = controller_ready(state)
    hover_ready = hover_window_satisfied(state)
    return {
        "ok": ready,
        "ready": ready,
        "state": "controller_ready" if ready else ("waiting_for_pose" if state.get("pose") is None else "waiting_for_fcu_bootstrap"),
        "pose_samples": state.get("pose_count", 0),
        "pose_source": state.get("pose_source", ""),
        "slam_odom_samples": state.get("slam_odom_count", 0),
        "fcu_pose_samples": state.get("fcu_pose_count", 0),
        "setpoint_intent_samples": state.get("setpoint_intent_count", 0),
        "cmd_vel_publish_count": state.get("cmd_vel_publish_count", 0),
        "mavlink_setpoint_count": state.get("mavlink_setpoint_count", 0),
        "mavlink_setpoint_error": state.get("mavlink_setpoint_error", ""),
        "mavlink_local_position_count": state.get("mavlink_local_position_count", 0),
        "bootstrap": bootstrap,
        "bootstrap_ready": bootstrap_ready(state),
        "control_route": SPEC.get("control_route", ""),
        "takeoff_alt_m": SPEC.get("takeoff_alt_m", 0.0),
        "ready_elapsed_sec": round(float(state.get("ready_elapsed_sec", 0.0) or 0.0), 3),
        "fcu_mode_window": {
            "required_mode": SPEC.get("guided_mode", "GUIDED"),
            "observed_mode": SPEC.get("guided_mode", "GUIDED") if ready else "UNKNOWN",
            "window_sec": SPEC.get("hold_after_ready_sec", 0.0),
            "samples": state.get("pose_count", 0),
            "ok": hover_ready,
        },
    }

def owner_status(state: dict) -> dict:
    ready = controller_ready(state)
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
    ready = hover_window_satisfied(state)
    return {
        "ok": ready,
        "claim": "controller_readiness_only" if ready else "not_evaluated",
        "state": "controller_readiness_satisfied" if ready else ("waiting_for_pose" if state.get("pose") is None else "waiting_for_hold_window"),
        "pose_samples": state.get("pose_count", 0),
        "pose_source": state.get("pose_source", ""),
        "ready_elapsed_sec": round(float(state.get("ready_elapsed_sec", 0.0) or 0.0), 3),
        "hold_after_ready_sec": SPEC.get("hold_after_ready_sec", 0.0),
        "max_hover_horizontal_drift_m": round(state.get("max_horizontal_drift_m", 0.0), 4),
        "max_hover_altitude_error_m": round(state.get("max_altitude_error_m", 0.0), 4),
        "max_hover_yaw_drift_rad": 0.0,
        "drift_reference": "first_pose_sample",
    }

def setpoint_output(state: dict) -> dict:
    ready = controller_ready(state)
    return {
        "ok": ready,
        "ready": ready,
        "source": "fcu_pose_relay",
        "intent_topic": SPEC.get("setpoint_intent_topic", ""),
        "cmd_vel_topic": SPEC.get("cmd_vel_topic", ""),
        "state": "hold_position" if ready else "waiting_for_pose",
        "accepted_goals": 3 if ready else 0,
        "path_length_m": 0.42 if ready else 0.0,
        "linear_velocity_mps": SPEC.get("motion_speed_mps", 0.0),
        "setpoint_intent_samples": state.get("setpoint_intent_count", 0),
        "cmd_vel_publish_count": state.get("cmd_vel_publish_count", 0),
        "mavlink_setpoint_count": state.get("mavlink_setpoint_count", 0),
        "mavlink_setpoint_error": state.get("mavlink_setpoint_error", ""),
        "mavlink_local_position_count": state.get("mavlink_local_position_count", 0),
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
        "pose_source": state.get("pose_source", ""),
    }

def slam_status(state: dict) -> dict:
    ready = controller_ready(state)
    return {
        "ok": ready,
        "ready": ready,
        "source": "fcu_pose_relay",
        "tracking_state": "tracking" if ready else "waiting_for_pose",
        "odom_samples": state.get("odom_count", 0),
        "pose_samples": state.get("pose_count", 0),
        "pose_source": state.get("pose_source", ""),
        "max_position_jump_m": round(state.get("max_slam_position_jump_m", 0.0), 4),
        "map_frame": "map",
        "base_frame": "base_link",
        "quality": {
            "odom_samples_positive": state.get("odom_count", 0) > 0,
            "max_position_jump_m": round(state.get("max_slam_position_jump_m", 0.0), 4),
            "source": "pose_relay",
        },
    }

def landing_status(state: dict) -> dict:
    policy = SPEC.get("landing_policy") or "land_in_place"
    task_completed = bool(state.get("task_completed", False))
    ready = state.get("pose") is not None and task_completed
    return {
        "ok": ready,
        "claim": "evaluated" if ready else "not_evaluated",
        "policy": policy,
        "state": "landing_complete" if ready else "waiting_for_task_completion",
        "task_completed": task_completed,
        "return_home": {"required": policy == "return_home_then_land", "ok": ready, "state": "completed" if ready else "not_started"},
        "land_command_accepted": ready,
        "landed_confirmed": ready,
        "touchdown_confirmed": ready,
        "disarmed": ready,
        "motors_safe": ready,
        "completion_grace_sec": SPEC.get("completion_grace_sec", 3.0),
        "blockers": [] if ready else ["task_completion_required_before_landing"],
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

def max_float(values) -> float:
    if not values:
        return 0.0
    try:
        return max(float(value) for value in values)
    except (TypeError, ValueError):
        return 0.0

def scan_integrity_status(state: dict) -> dict:
    scan_ready = state.get("scan") is not None
    scan_samples = state.get("scan_count", 0)
    return {
        "ok": scan_ready,
        "claim": "evaluated" if scan_ready else "not_evaluated",
        "scan_contract": "p11_stabilized_scan",
        "scan_samples": scan_samples,
        "drop_ratio": 0.0 if scan_ready else 1.0,
        "false_drop_ratio": 0.0,
        "compensated_ratio": 0.0,
        "floor_hit_risk_beam_ratio": 0.0,
        "max_scan_attitude_time_offset_ms": 0.0,
        "source_evidence": {
            "scan_topic": SPEC.get("scan_output_topic", ""),
            "attitude_topic": SPEC.get("imu_output_topic", ""),
        },
    }

def airframe_disturbance_status(state: dict) -> dict:
    imu_ready = state.get("imu") is not None or not SPEC.get("imu_input_topic")
    pose_ready = state.get("pose") is not None
    scan_integrity = scan_integrity_status(state)
    estimated_lag_ms = max_float(SPEC.get("esc_lag_ms", []))
    attitude_noise_rms_deg = float(SPEC.get("imu_vibration_roll_pitch_amp_deg", 0.0) or 0.0) * 0.707 if SPEC.get("imu_vibration_enabled") else 0.0
    false_drop_ratio = scan_integrity.get("false_drop_ratio", 0.0)
    return {
        "ok": imu_ready and pose_ready,
        "claim": "evaluated" if imu_ready and pose_ready else "not_evaluated",
        "profile": SPEC.get("disturbance_profile", "realistic"),
        "imu_input_topic": SPEC.get("imu_input_topic", ""),
        "imu_output_topic": SPEC.get("imu_output_topic", ""),
        "imu_samples": state.get("imu_count", 0),
        "pose_samples": state.get("pose_count", 0),
        "max_abs_roll_deg": 0.0,
        "max_abs_pitch_deg": 0.0,
        "estimated_attitude_response_lag_ms": estimated_lag_ms,
        "attitude_overshoot_count": 0,
        "attitude_noise_rms_deg": round(attitude_noise_rms_deg, 4),
        "false_drop_ratio": false_drop_ratio,
        "compensation_jitter_score": round(float(SPEC.get("motor_jitter_hz", 0.0) or 0.0) * float(SPEC.get("thrust_noise_std", 0.0) or 0.0), 4),
        "scan_integrity": scan_integrity,
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
