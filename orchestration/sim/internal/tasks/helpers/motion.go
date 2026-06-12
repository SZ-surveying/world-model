package helpers

import (
	"crypto/sha256"
	"encoding/hex"
	"os"
)

const MotionRosbagContainer = "navlab-motion-rosbag"

type MotionGateSpec struct {
	RuntimeConfigPath      string
	RosbagProfile          string
	SlamOdomTopic          string
	CanonicalSlamOdomTopic string
	ExternalNavStatusTopic string
	MotionStatusTopic      string
	CmdVelTopic            string
	CanonicalCmdVelTopic   string
	SetpointOutputTopic    string
	FCUPoseTopic           string
	FCUTwistTopic          string
	TruthDiagnosticTopic   string
	UsesGazeboTruthAsInput bool
	HoverClaim             string
	MotionClaim            string
	ExplorationClaim       string
	MotionDistanceM        float64
	MotionSpeedMPS         float64
	YawScanRad             float64
	MaxStopDriftM          float64
	MinClearanceM          float64
}

type DependencyDoctor struct {
	OK       bool     `json:"ok"`
	Blockers []string `json:"blockers"`
	Skipped  string   `json:"skipped,omitempty"`
}

type MotionDoctorSummary struct {
	OK                    bool             `json:"ok"`
	Blocked               bool             `json:"blocked"`
	Blockers              []string         `json:"blockers"`
	MotionGateDoctor      MotionGateDoctor `json:"motion_gate_doctor"`
	HoverDependencyDoctor DependencyDoctor `json:"hover_dependency_doctor"`
}

type MotionGateDoctor struct {
	RuntimeConfig            string              `json:"runtime_config"`
	RuntimeConfigSHA256      string              `json:"runtime_config_sha256"`
	DependencyChecksIncluded bool                `json:"dependency_checks_included"`
	SlamOdomTopic            string              `json:"slam_odom_topic"`
	ExternalNavStatusTopic   string              `json:"external_nav_status_topic"`
	MotionStatusTopic        string              `json:"motion_status_topic"`
	UsesGazeboTruthAsInput   bool                `json:"uses_gazebo_truth_as_input"`
	HoverClaim               string              `json:"hover_claim"`
	MotionClaim              string              `json:"motion_claim"`
	ExplorationClaim         string              `json:"exploration_claim"`
	Thresholds               MotionThresholds    `json:"thresholds"`
	RosbagProfile            MotionRosbagProfile `json:"rosbag_profile"`
}

type MotionThresholds struct {
	MotionDistanceM float64 `json:"motion_distance_m"`
	MotionSpeedMPS  float64 `json:"motion_speed_mps"`
	YawScanRad      float64 `json:"yaw_scan_rad"`
	MaxStopDriftM   float64 `json:"max_stop_drift_m"`
	MinClearanceM   float64 `json:"min_clearance_m"`
}

type MotionRosbagProfile struct {
	Profile        string   `json:"profile"`
	RequiredTopics []string `json:"required_topics"`
	OptionalTopics []string `json:"optional_topics"`
}

func DefaultMotionGateSpec() MotionGateSpec {
	return MotionGateSpec{
		RuntimeConfigPath:      "motion_gate_runtime.toml",
		RosbagProfile:          "",
		SlamOdomTopic:          "/slam/odom",
		CanonicalSlamOdomTopic: "/slam/odom",
		ExternalNavStatusTopic: "/external_nav/status",
		MotionStatusTopic:      "/navlab/motion/status",
		CmdVelTopic:            "/ap/cmd_vel",
		CanonicalCmdVelTopic:   "/ap/cmd_vel",
		SetpointOutputTopic:    "/navlab/setpoint/output",
		FCUPoseTopic:           "/ap/pose/filtered",
		FCUTwistTopic:          "/ap/twist/filtered",
		TruthDiagnosticTopic:   "/navlab/truth/odom",
		UsesGazeboTruthAsInput: false,
		HoverClaim:             "evaluated",
		MotionClaim:            "evaluated",
		ExplorationClaim:       "not_evaluated",
		MotionDistanceM:        0.5,
		MotionSpeedMPS:         0.1,
		YawScanRad:             0.6,
		MaxStopDriftM:          0.2,
		MinClearanceM:          0.35,
	}
}

func BuildMotionDoctorSummary(spec MotionGateSpec, dependency DependencyDoctor, includeDependencies bool) MotionDoctorSummary {
	if spec.RuntimeConfigPath == "" {
		spec = DefaultMotionGateSpec()
	}
	if !includeDependencies {
		dependency = DependencyDoctor{
			OK:       true,
			Blockers: []string{},
			Skipped:  "acceptance already launched hover prerequisites",
		}
	}
	blockers := append([]string{}, dependency.Blockers...)
	topics := Topics{}
	if spec.RosbagProfile == "" {
		blockers = append(blockers, "motion rosbag profile is missing or empty")
	} else if loaded, err := ProfileTopics(spec.RosbagProfile); err != nil || len(loaded.All) == 0 {
		blockers = append(blockers, "motion rosbag profile is missing or empty")
	} else {
		topics = loaded
	}
	if spec.UsesGazeboTruthAsInput {
		blockers = append(blockers, "motion gate must not use Gazebo truth as a control/planning/SLAM/ExternalNav input")
	}
	if spec.SlamOdomTopic != spec.CanonicalSlamOdomTopic {
		blockers = append(blockers, "motion gate SLAM odom topic must match canonical SLAM odom topic")
	}
	if spec.SlamOdomTopic == spec.TruthDiagnosticTopic {
		blockers = append(blockers, "motion gate SLAM odom topic must not be the Gazebo truth diagnostic topic")
	}
	if spec.CmdVelTopic != spec.CanonicalCmdVelTopic {
		blockers = append(blockers, "motion gate cmd_vel topic must match the FCU controller output topic")
	}
	blockers = uniqueSorted(blockers)
	return MotionDoctorSummary{
		OK:       len(blockers) == 0,
		Blocked:  len(blockers) > 0,
		Blockers: blockers,
		MotionGateDoctor: MotionGateDoctor{
			RuntimeConfig:            spec.RuntimeConfigPath,
			RuntimeConfigSHA256:      fileHashIfExists(spec.RuntimeConfigPath),
			DependencyChecksIncluded: includeDependencies,
			SlamOdomTopic:            spec.SlamOdomTopic,
			ExternalNavStatusTopic:   spec.ExternalNavStatusTopic,
			MotionStatusTopic:        spec.MotionStatusTopic,
			UsesGazeboTruthAsInput:   spec.UsesGazeboTruthAsInput,
			HoverClaim:               spec.HoverClaim,
			MotionClaim:              spec.MotionClaim,
			ExplorationClaim:         spec.ExplorationClaim,
			Thresholds: MotionThresholds{
				MotionDistanceM: spec.MotionDistanceM,
				MotionSpeedMPS:  spec.MotionSpeedMPS,
				YawScanRad:      spec.YawScanRad,
				MaxStopDriftM:   spec.MaxStopDriftM,
				MinClearanceM:   spec.MinClearanceM,
			},
			RosbagProfile: MotionRosbagProfile{
				Profile:        spec.RosbagProfile,
				RequiredTopics: append([]string{}, topics.Required...),
				OptionalTopics: append([]string{}, topics.Optional...),
			},
		},
		HoverDependencyDoctor: dependency,
	}
}

func MotionFoxgloveNotes(spec MotionGateSpec) string {
	return "# NavLab motion gate replay notes\n\n" +
		"The motion gate validates forward/back/yaw/stop motion after SLAM hover. It is not an exploration gate.\n\n" +
		"- Fixed frame: `map`.\n" +
		"- Motion status: `" + spec.MotionStatusTopic + "`.\n" +
		"- Setpoint output: `" + spec.SetpointOutputTopic + "`.\n" +
		"- SLAM odom: `" + spec.SlamOdomTopic + "`.\n" +
		"- FCU pose/twist: `" + spec.FCUPoseTopic + "`, `" + spec.FCUTwistTopic + "`.\n" +
		"- Diagnostic truth only: `" + spec.TruthDiagnosticTopic + "`.\n" +
		"- Do not use Gazebo truth as a SLAM, ExternalNav, planning, or control input.\n"
}

func fileHashIfExists(path string) string {
	if path == "" {
		return ""
	}
	data, err := os.ReadFile(path)
	if err != nil {
		return ""
	}
	sum := sha256.Sum256(data)
	return hex.EncodeToString(sum[:])
}
