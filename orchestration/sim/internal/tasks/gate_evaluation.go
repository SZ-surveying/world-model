package tasks

import (
	"encoding/json"
	"fmt"
	"os"
	"path/filepath"
	"sort"
	"strings"
	"time"

	"navlab/orchestration-sim/internal/config"
	"navlab/orchestration-sim/internal/tasks/helpers"
)

type GateEvaluation struct {
	OK             bool                 `json:"ok"`
	Blocked        bool                 `json:"blocked"`
	Blockers       []string             `json:"blockers"`
	ProbeOutputs   []ProbeOutputSummary `json:"probe_outputs"`
	RosbagProfiles []RosbagGateSummary  `json:"rosbag_profiles"`
	Landing        helpers.Acceptance   `json:"landing_acceptance"`
	TaskChecks     map[string]bool      `json:"task_checks"`
	Metrics        MetricSummary        `json:"metrics"`
}

type ProbeOutputSummary struct {
	Name    string         `json:"name"`
	Path    string         `json:"path"`
	Exists  bool           `json:"exists"`
	OK      bool           `json:"ok"`
	Status  string         `json:"status,omitempty"`
	Payload map[string]any `json:"payload,omitempty"`
}

type RosbagGateSummary struct {
	Name                        string         `json:"name"`
	MetadataPath                string         `json:"metadata_path"`
	Exists                      bool           `json:"exists"`
	OK                          bool           `json:"ok"`
	RequiredTopics              []string       `json:"required_topics"`
	MessageCounts               map[string]int `json:"message_counts,omitempty"`
	MissingRequiredTopics       []string       `json:"missing_required_topics,omitempty"`
	ZeroCountRequiredTopics     []string       `json:"zero_count_required_topics,omitempty"`
	MetadataMissingOrUnreadable string         `json:"metadata_missing_or_unreadable,omitempty"`
}

type MetricSummary struct {
	Hover                 map[string]any            `json:"hover,omitempty"`
	HoverMission          map[string]any            `json:"hover_mission,omitempty"`
	Controller            map[string]any            `json:"controller,omitempty"`
	Setpoint              map[string]any            `json:"setpoint,omitempty"`
	Owner                 map[string]any            `json:"owner,omitempty"`
	Exploration           map[string]any            `json:"exploration,omitempty"`
	Nav2                  map[string]any            `json:"nav2,omitempty"`
	Navigation            map[string]any            `json:"navigation,omitempty"`
	NavigationAdapter     map[string]any            `json:"navigation_adapter,omitempty"`
	CostmapHealth         map[string]any            `json:"costmap_health,omitempty"`
	SLAM                  map[string]any            `json:"slam,omitempty"`
	ExternalNav           map[string]any            `json:"external_nav,omitempty"`
	MAVLinkExternalNav    map[string]any            `json:"mavlink_external_nav,omitempty"`
	SLAMRuntimeLog        map[string]any            `json:"slam_runtime_log,omitempty"`
	ScanIntegrity         map[string]any            `json:"scan_integrity,omitempty"`
	ScanStabilization     map[string]any            `json:"scan_stabilization,omitempty"`
	AirframeDisturbance   map[string]any            `json:"airframe_disturbance,omitempty"`
	RosbagMessageCounts   map[string]map[string]int `json:"rosbag_message_counts,omitempty"`
	MetricEvidenceSources map[string]string         `json:"metric_evidence_sources,omitempty"`
}

func EvaluateResultGates(
	project config.ProjectConfig,
	runtimeConfig config.TaskRuntimeConfig,
	plan Plan,
	artifactDir string,
	runtimeSpecs RuntimeSpecBundle,
	execution RuntimeExecutionResult,
	executionErr error,
) GateEvaluation {
	blockers := []string{}
	taskChecks := taskSpecificChecks(runtimeConfig, plan.TaskID)
	for name, ok := range taskChecks {
		if !ok {
			blockers = append(blockers, "task_check_failed:"+name)
		}
	}
	blockers = append(blockers, taskSpecificBlockers(runtimeConfig, plan.TaskID)...)
	if executionErr != nil {
		blockers = append(blockers, "runtime_execution_failed")
	}
	for _, result := range execution.ProbeResults {
		if !result.OK() {
			blockers = append(blockers, fmt.Sprintf("probe_failed:%s:rc=%d", result.Name, result.ReturnCode))
		}
	}

	probeOutputs := evaluateProbeOutputs(plan, artifactDir)
	for _, probe := range probeOutputs {
		if !probe.OK {
			if !probe.Exists {
				blockers = append(blockers, "probe_output_missing:"+probe.Name)
			} else {
				blockers = append(blockers, "probe_output_not_ok:"+probe.Name)
			}
		}
		blockers = append(blockers, probeBlockers(probe)...)
	}

	rosbagProfiles := evaluateRosbagProfiles(plan, artifactDir)
	for _, rosbag := range rosbagProfiles {
		if !rosbag.OK {
			blockers = append(blockers, "rosbag_profile_failed:"+rosbag.Name)
		}
	}

	landingEvidence := landingFromProbeOutputs(probeOutputs)
	landing := helpers.BuildAcceptance(
		"simulation",
		landingConfig(project, runtimeConfig, plan.TaskID),
		landingEvidence,
		false,
	)
	blockers = append(blockers, landing.Blockers...)
	metrics := metricSummaryFromEvidence(probeOutputs, rosbagProfiles, artifactDir)
	blockers = append(blockers, slamRuntimeLogBlockers(metrics.SLAMRuntimeLog)...)
	if plan.TaskID == "hover" {
		blockers = append(blockers, externalNavFeedbackBlockers(metrics.ExternalNav, metrics.MAVLinkExternalNav)...)
		blockers = append(blockers, hoverMissionBlockers(metrics.HoverMission)...)
		if len(metrics.Controller) > 0 {
			blockers = append(blockers, hoverTakeoffBlockers(plan.TaskID, metrics.Controller)...)
		}
	}

	blockers = uniqueStrings(blockers)
	return GateEvaluation{
		OK:             len(blockers) == 0,
		Blocked:        len(blockers) > 0,
		Blockers:       blockers,
		ProbeOutputs:   probeOutputs,
		RosbagProfiles: rosbagProfiles,
		Landing:        landing,
		TaskChecks:     taskChecks,
		Metrics:        metrics,
	}
}

func metricSummaryFromEvidence(probes []ProbeOutputSummary, rosbags []RosbagGateSummary, artifactDirs ...string) MetricSummary {
	summary := MetricSummary{
		MetricEvidenceSources: map[string]string{},
	}
	if payload, topic := statusPayloadBySuffix(probes, "/hover/status"); payload != nil {
		summary.Hover = subsetMap(payload, "claim", "state", "phase", "reason", "pose_samples", "setpoints_sent_count", "local_position_count", "position", "max_hover_horizontal_drift_m", "max_hover_altitude_error_m", "max_hover_yaw_drift_rad", "drift_reference")
		summary.MetricEvidenceSources["hover"] = topic
	}
	if payload, topic := statusPayloadBySuffix(probes, "/fcu/controller/status"); payload != nil {
		summary.Controller = subsetMap(payload, "ready", "state", "pose_samples", "control_route", "takeoff_alt_m", "fcu_mode_window", "cmd_vel_publish_count", "mavlink_setpoint_count", "mavlink_setpoint_error", "mavlink_local_position_count", "bootstrap_ready", "bootstrap")
		summary.MetricEvidenceSources["controller"] = topic
	}
	if payload, topic := statusPayloadBySuffix(probes, "/fcu/setpoint/output"); payload != nil {
		summary.Setpoint = subsetMap(payload, "ready", "state", "intent_topic", "cmd_vel_topic", "setpoint_intent_samples", "cmd_vel_publish_count", "mavlink_setpoint_count", "mavlink_setpoint_error", "mavlink_local_position_count", "path_length_m", "min_path_length_m")
		summary.MetricEvidenceSources["setpoint"] = topic
	}
	if payload, topic := statusPayloadBySuffix(probes, "/fcu/owner/status"); payload != nil {
		summary.Owner = subsetMap(payload, "active", "owner", "active_owner_count", "owner_unique", "conflicting_owners")
		summary.MetricEvidenceSources["owner"] = topic
	}
	if payload, topic := statusPayloadBySuffix(probes, "/exploration/status"); payload != nil {
		summary.Exploration = subsetMap(payload, "claim", "strategy", "accepted_goals", "min_accepted_goals", "path_length_m", "min_path_length_m", "motion_speed_mps")
		summary.MetricEvidenceSources["exploration"] = topic
	}
	if payload, source := probePayloadByName(probes, "nav2_lifecycle_probe"); payload != nil {
		summary.Nav2 = subsetMap(payload, "claim", "nav2_claim", "nav2_lifecycle_active", "nav2_action_server_ready", "lifecycle", "action", "tf")
		summary.MetricEvidenceSources["nav2"] = source
	}
	if payload, topic := statusPayloadBySuffix(probes, "/navigation/status"); payload != nil {
		summary.Navigation = subsetMap(payload, "claim", "navigation_claim", "frontier_claim", "strategy", "frontier_candidates", "accepted_frontiers", "rejected_frontiers", "blacklisted_goals", "accepted_goals", "min_accepted_goals", "path_length_m", "min_path_length_m", "recovery_count", "completion_policy", "return_home_policy", "goal_success_ratio", "coverage_growth", "min_coverage_growth", "uses_gazebo_truth_as_input")
		summary.MetricEvidenceSources["navigation"] = topic
	}
	if payload, topic := statusPayloadBySuffix(probes, "/navigation/adapter/status"); payload != nil {
		summary.NavigationAdapter = subsetMap(payload, "claim", "adapter_claim", "active", "max_xy_speed_mps", "fixed_altitude_m", "stop_on_stale_costmap", "stop_on_stale_slam", "clamp_count", "hold_count", "intent_count", "hold_reason")
		summary.MetricEvidenceSources["navigation_adapter"] = topic
	}
	if payload, topic := statusPayloadBySuffix(probes, "/navigation/costmap_health"); payload != nil {
		summary.CostmapHealth = subsetMap(payload, "claim", "costmap_claim", "global_costmap_age_sec", "local_costmap_age_sec", "local_costmap_update_frequency_hz", "unknown_ratio", "obstacle_cells", "required_layers")
		summary.MetricEvidenceSources["costmap_health"] = topic
	}
	if payload, topic := statusPayloadBySuffix(probes, "/slam/status"); payload != nil {
		summary.SLAM = subsetMap(payload, "state", "ready", "mode", "tracking_state", "odom_samples", "pose_samples", "max_position_jump_m", "map_frame", "base_frame", "quality", "scan", "imu", "tf", "output")
		summary.MetricEvidenceSources["slam"] = topic
	}
	if payload, topic := statusPayloadBySuffix(probes, "/external_nav/status"); payload != nil {
		summary.ExternalNav = subsetMap(payload, "state", "ready", "odom", "imu", "height", "output")
		summary.MetricEvidenceSources["external_nav"] = topic
	}
	if payload, topic := statusPayloadBySuffix(probes, "/mavlink_external_nav/status"); payload != nil {
		summary.MAVLinkExternalNav = subsetMap(payload, "state", "ready", "endpoint", "input_topic", "sent_count", "rate_hz", "odom_age_ms", "max_odom_age_ms", "odom_fresh", "frame_id", "child_frame_id", "quality", "use_fcu_roll_pitch", "fcu_attitude_age_ms", "local_position_pose_topic", "local_position_count", "local_position_age_ms", "max_local_position_age_ms", "fcu_local_position_ready")
		summary.MetricEvidenceSources["mavlink_external_nav"] = topic
	}
	if len(artifactDirs) > 0 && strings.TrimSpace(artifactDirs[0]) != "" {
		if metrics := parseHoverMissionSummary(filepath.Join(artifactDirs[0], "mission_summary.json")); metrics != nil {
			summary.HoverMission = metrics
			summary.MetricEvidenceSources["hover_mission"] = "mission_summary.json"
		}
		if metrics := parseSLAMRuntimeLog(filepath.Join(artifactDirs[0], "slam_backend.runtime.log")); metrics != nil {
			summary.SLAMRuntimeLog = metrics
			summary.MetricEvidenceSources["slam_runtime_log"] = "slam_backend.runtime.log"
		}
	}
	if payload, topic := statusPayloadBySuffix(probes, "/scan_integrity/status"); payload != nil {
		summary.ScanIntegrity = subsetMap(payload, "claim", "scan_contract", "scan_samples", "drop_ratio", "false_drop_ratio", "compensated_ratio", "floor_hit_risk_beam_ratio", "max_scan_attitude_time_offset_ms", "source_evidence")
		summary.MetricEvidenceSources["scan_integrity"] = topic
	}
	if payload, topic := statusPayloadBySuffix(probes, "/scan_stabilization/status"); payload != nil {
		summary.ScanStabilization = subsetMap(payload, "claim", "mode", "scan_samples", "imu_samples", "retained_beam_ratio", "rejected_beam_ratio", "floor_hit_risk_beam_ratio", "max_vertical_projection_error_m")
		summary.MetricEvidenceSources["scan_stabilization"] = topic
	}
	if payload, topic := statusPayloadBySuffix(probes, "/airframe_disturbance/status"); payload != nil {
		summary.AirframeDisturbance = subsetMap(payload, "claim", "profile", "imu_samples", "pose_samples", "max_abs_roll_deg", "max_abs_pitch_deg", "estimated_attitude_response_lag_ms", "attitude_overshoot_count", "attitude_noise_rms_deg", "false_drop_ratio", "compensation_jitter_score", "scan_integrity", "profile_sweep", "fcu_mode_window")
		summary.MetricEvidenceSources["airframe_disturbance"] = topic
	}
	for _, rosbag := range rosbags {
		if len(rosbag.MessageCounts) == 0 {
			continue
		}
		if summary.RosbagMessageCounts == nil {
			summary.RosbagMessageCounts = map[string]map[string]int{}
		}
		summary.RosbagMessageCounts[rosbag.Name] = rosbag.MessageCounts
	}
	if len(summary.MetricEvidenceSources) == 0 {
		summary.MetricEvidenceSources = nil
	}
	return summary
}

func parseHoverMissionSummary(path string) map[string]any {
	var data []byte
	var err error
	for attempt := 0; attempt < 50; attempt++ {
		data, err = os.ReadFile(path)
		if err != nil {
			return nil
		}
		payload := map[string]any{}
		if err := json.Unmarshal(data, &payload); err == nil {
			payload["path"] = path
			payload["exists"] = true
			return subsetHoverMissionSummary(payload)
		}
		time.Sleep(100 * time.Millisecond)
	}
	payload := map[string]any{}
	if err := json.Unmarshal(data, &payload); err != nil {
		return map[string]any{"path": path, "exists": true, "parse_error": err.Error(), "ok": false}
	}
	payload["path"] = path
	payload["exists"] = true
	return subsetHoverMissionSummary(payload)
}

func subsetHoverMissionSummary(payload map[string]any) map[string]any {
	return subsetMap(payload,
		"path",
		"exists",
		"ok",
		"reason",
		"hover_body_ok",
		"landing_ok",
		"phases_seen",
		"phase_counts",
		"guided_seen",
		"armed_seen",
		"takeoff_ack_ok",
		"airborne_seen",
		"local_position_count",
		"setpoints_sent_count",
		"target_alt_m",
		"takeoff_alt_m",
		"current_z_ned",
		"last_position",
		"target_z_ned",
		"altitude_error_m",
		"hover_altitude_tolerance_m",
		"hover_hold_sec",
		"hover_hold_duration_sec",
		"require_external_nav",
		"external_nav_ready",
		"external_nav_status_age_sec",
		"external_nav_status_history",
		"land_command_accepted",
		"touchdown_confirmed",
		"disarmed",
		"motors_safe",
		"hover_drift",
		"landing",
		"parse_error",
	)
}

func externalNavFeedbackBlockers(externalNav map[string]any, mavlinkExternalNav map[string]any) []string {
	blockers := []string{}
	if len(externalNav) == 0 {
		blockers = append(blockers, "external_nav_status_missing")
	} else {
		if ready, _ := externalNav["ready"].(bool); !ready {
			blockers = append(blockers, "external_nav_bridge_not_ready")
		}
		odom := mapFromAny(externalNav["odom"])
		inputTopic, _ := odom["input_topic"].(string)
		inputTopic = strings.TrimSpace(inputTopic)
		if inputTopic == "" {
			blockers = append(blockers, "external_nav_input_topic_missing")
		} else {
			if inputTopic != "/slam/odom" {
				blockers = append(blockers, "external_nav_not_using_slam_odom")
			}
			if strings.Contains(inputTopic, "gazebo") || inputTopic == "/odometry" {
				blockers = append(blockers, "external_nav_uses_diagnostic_truth_input")
			}
		}
	}
	if len(mavlinkExternalNav) == 0 {
		blockers = append(blockers, "mavlink_external_nav_status_missing")
		return blockers
	}
	if state, _ := mavlinkExternalNav["state"].(string); state != "sending" {
		blockers = append(blockers, "mavlink_external_nav_not_sending")
	}
	if ready, _ := mavlinkExternalNav["ready"].(bool); !ready {
		blockers = append(blockers, "mavlink_external_nav_not_ready")
	}
	if metricInt(mavlinkExternalNav, "sent_count") <= 0 {
		blockers = append(blockers, "mavlink_external_nav_not_sent")
	}
	if inputTopic, _ := mavlinkExternalNav["input_topic"].(string); strings.TrimSpace(inputTopic) != "/external_nav/odom" {
		blockers = append(blockers, "mavlink_external_nav_input_topic_invalid")
	}
	if metricInt(mavlinkExternalNav, "local_position_count") <= 0 {
		blockers = append(blockers, "external_nav_not_seen_by_fcu")
	}
	if fcuReady, ok := mavlinkExternalNav["fcu_local_position_ready"].(bool); ok && !fcuReady {
		blockers = append(blockers, "external_nav_fcu_local_position_not_ready")
	}
	localPositionAgeMS := metricFloat(mavlinkExternalNav, "local_position_age_ms")
	maxLocalPositionAgeMS := metricFloat(mavlinkExternalNav, "max_local_position_age_ms")
	if maxLocalPositionAgeMS <= 0 {
		maxLocalPositionAgeMS = 1000
	}
	if localPositionAgeMS < 0 || localPositionAgeMS > maxLocalPositionAgeMS {
		blockers = append(blockers, "external_nav_fcu_local_position_stale")
	}
	return blockers
}

func hoverMissionBlockers(mission map[string]any) []string {
	if len(mission) == 0 {
		return []string{"hover_mission_summary_missing"}
	}
	if parseError, _ := mission["parse_error"].(string); strings.TrimSpace(parseError) != "" {
		return []string{"hover_mission_summary_unreadable"}
	}
	blockers := []string{}
	if ok, _ := mission["ok"].(bool); !ok {
		blockers = append(blockers, "hover_mission_not_ok")
	}
	if requireExternalNav, _ := mission["require_external_nav"].(bool); requireExternalNav {
		if externalNavReady, _ := mission["external_nav_ready"].(bool); !externalNavReady {
			blockers = append(blockers, "hover_mission_external_nav_not_ready")
		}
	}
	if !stringAnySliceContains(mission["phases_seen"], "hover_hold") {
		blockers = append(blockers, "hover_mission_hover_hold_missing")
	}
	for _, field := range []string{"guided_seen", "armed_seen", "airborne_seen"} {
		if value, _ := mission[field].(bool); !value {
			blockers = append(blockers, "hover_mission_"+field+"_missing")
		}
	}
	if metricInt(mission, "local_position_count") <= 0 {
		blockers = append(blockers, "hover_mission_local_position_missing")
	}
	if metricInt(mission, "setpoints_sent_count") <= 0 {
		blockers = append(blockers, "hover_mission_setpoints_missing")
	}
	targetAlt := metricFloat(mission, "target_alt_m")
	if targetAlt <= 0 {
		targetAlt = metricFloat(mission, "takeoff_alt_m")
	}
	if targetAlt <= 0 || !metricNumberPresent(mission, "current_z_ned") || !metricNumberPresent(mission, "altitude_error_m") {
		blockers = append(blockers, "hover_mission_altitude_evidence_missing")
	} else {
		altitudeError := metricFloat(mission, "altitude_error_m")
		tolerance := metricFloat(mission, "hover_altitude_tolerance_m")
		if tolerance <= 0 {
			tolerance = 0.18
		}
		if altitudeError > tolerance {
			blockers = append(blockers, "hover_mission_target_altitude_not_reached")
		}
	}
	if takeoffAckOK, _ := mission["takeoff_ack_ok"].(bool); !takeoffAckOK && !hoverMissionHasTakeoffEvidence(mission) {
		blockers = append(blockers, "hover_mission_takeoff_ack_missing")
	}
	hoverDrift := mapFromAny(mission["hover_drift"])
	hoverHoldSec := metricFloat(mission, "hover_hold_sec")
	durationTolerance := metricFloat(hoverDrift, "duration_tolerance_sec")
	if durationTolerance <= 0 {
		durationTolerance = 0.25
	}
	if len(hoverDrift) == 0 {
		blockers = append(blockers, "hover_mission_drift_evidence_missing")
	} else {
		if metricInt(hoverDrift, "sample_count") < 2 {
			blockers = append(blockers, "hover_mission_hold_samples_missing")
		}
		if hoverHoldSec > 0 && metricFloat(hoverDrift, "duration_sec") < hoverHoldSec-durationTolerance {
			blockers = append(blockers, "hover_mission_hold_duration_short")
		}
		if driftOK, _ := hoverDrift["ok"].(bool); !driftOK {
			blockers = append(blockers, "hover_mission_drift_not_ok")
		}
	}
	if hoverBodyOK, _ := mission["hover_body_ok"].(bool); !hoverBodyOK {
		blockers = append(blockers, "hover_mission_body_not_ok")
	}
	if landingOK, _ := mission["landing_ok"].(bool); !landingOK {
		blockers = append(blockers, "hover_mission_landing_not_ok")
	}
	landing := mapFromAny(mission["landing"])
	if len(landing) == 0 {
		landing = mission
	}
	for _, field := range []string{"touchdown_confirmed", "disarmed", "motors_safe"} {
		if value, _ := landing[field].(bool); !value {
			blockers = append(blockers, "hover_mission_landing_"+field+"_missing")
		}
	}
	return blockers
}

func hoverMissionHasTakeoffEvidence(mission map[string]any) bool {
	if airborneSeen, _ := mission["airborne_seen"].(bool); !airborneSeen {
		return false
	}
	if metricInt(mission, "local_position_count") <= 0 {
		return false
	}
	if !metricNumberPresent(mission, "current_z_ned") || !metricNumberPresent(mission, "altitude_error_m") {
		return false
	}
	tolerance := metricFloat(mission, "hover_altitude_tolerance_m")
	if tolerance <= 0 {
		tolerance = 0.18
	}
	return metricFloat(mission, "altitude_error_m") <= tolerance
}

func parseSLAMRuntimeLog(path string) map[string]any {
	data, err := os.ReadFile(path)
	if err != nil {
		return nil
	}
	lines := strings.Split(string(data), "\n")
	metrics := map[string]any{
		"path":                         path,
		"exists":                       true,
		"dropped_earlier_points_count": 0,
		"rejected_odom_tf_log_count":   0,
		"std_length_error_count":       0,
		"fatal_count":                  0,
		"error_count":                  0,
		"warning_count":                0,
	}
	problemLines := []string{}
	for _, line := range lines {
		trimmed := strings.TrimSpace(line)
		if trimmed == "" {
			continue
		}
		lower := strings.ToLower(trimmed)
		matched := false
		if strings.Contains(trimmed, "Dropped earlier points") {
			incrementMetric(metrics, "dropped_earlier_points_count")
			matched = true
		}
		if strings.Contains(trimmed, "rejected odom TF") {
			incrementMetric(metrics, "rejected_odom_tf_log_count")
			matched = true
		}
		if strings.Contains(trimmed, "std::length_error") {
			incrementMetric(metrics, "std_length_error_count")
			matched = true
		}
		if strings.Contains(lower, "fatal") || hasGlogSeverityPrefix(trimmed, 'F') {
			incrementMetric(metrics, "fatal_count")
			matched = true
		}
		if strings.Contains(lower, "error") || hasGlogSeverityPrefix(trimmed, 'E') {
			incrementMetric(metrics, "error_count")
			matched = true
		}
		if strings.Contains(lower, "warn") || hasGlogSeverityPrefix(trimmed, 'W') {
			incrementMetric(metrics, "warning_count")
			matched = true
		}
		if matched {
			problemLines = appendLimited(problemLines, trimmed, 8)
		}
	}
	metrics["ok"] = metrics["std_length_error_count"] == 0 && metrics["fatal_count"] == 0
	if len(problemLines) > 0 {
		metrics["last_problem_lines"] = problemLines
	}
	return metrics
}

func slamRuntimeLogBlockers(metrics map[string]any) []string {
	if len(metrics) == 0 {
		return nil
	}
	if ok, _ := metrics["ok"].(bool); ok {
		return nil
	}
	blockers := []string{"slam_runtime_unhealthy"}
	if metricInt(metrics, "std_length_error_count") > 0 {
		blockers = append(blockers, "slam_runtime_std_length_error")
	}
	if metricInt(metrics, "fatal_count") > 0 {
		blockers = append(blockers, "slam_runtime_fatal")
	}
	if metricInt(metrics, "error_count") > 0 {
		blockers = append(blockers, "slam_runtime_error")
	}
	return blockers
}

func hoverTakeoffBlockers(taskID string, controller map[string]any) []string {
	if taskID != "hover" {
		return nil
	}
	if len(controller) == 0 {
		return []string{"hover_takeoff_evidence_missing"}
	}
	bootstrap := mapFromAny(controller["bootstrap"])
	takeoff := mapFromAny(bootstrap["takeoff"])
	if len(takeoff) == 0 {
		return []string{"hover_takeoff_evidence_missing"}
	}
	if ok, _ := takeoff["ok"].(bool); !ok {
		return []string{"hover_takeoff_not_ok"}
	}
	if ackAccepted, _ := takeoff["ack_accepted_seen"].(bool); !ackAccepted {
		return []string{"hover_takeoff_ack_missing"}
	}
	height := mapFromAny(takeoff["height"])
	observed := metricFloat(height, "height_m")
	required := hoverRequiredTakeoffHeightM(controller, height)
	if observed <= 0 || required <= 0 {
		return []string{"hover_takeoff_height_not_observed"}
	}
	if observed < required {
		return []string{"hover_takeoff_target_altitude_not_reached"}
	}
	return nil
}

func hoverRequiredTakeoffHeightM(controller map[string]any, height map[string]any) float64 {
	takeoffAlt := metricFloat(controller, "takeoff_alt_m")
	if takeoffAlt > 0 {
		required := takeoffAlt * 0.80
		if required < 0.40 {
			required = 0.40
		}
		if required > takeoffAlt {
			required = takeoffAlt
		}
		return required
	}
	return metricFloat(height, "target_min_m")
}

func metricInt(metrics map[string]any, key string) int {
	switch value := metrics[key].(type) {
	case int:
		return value
	case int64:
		return int(value)
	case float64:
		return int(value)
	default:
		return 0
	}
}

func metricFloat(metrics map[string]any, key string) float64 {
	switch value := metrics[key].(type) {
	case int:
		return float64(value)
	case int64:
		return float64(value)
	case float64:
		return value
	default:
		return 0
	}
}

func metricNumberPresent(metrics map[string]any, key string) bool {
	switch metrics[key].(type) {
	case int, int64, float64:
		return true
	default:
		return false
	}
}

func mapFromAny(value any) map[string]any {
	if typed, ok := value.(map[string]any); ok {
		return typed
	}
	return nil
}

func hasGlogSeverityPrefix(line string, severity byte) bool {
	if len(line) < 2 || line[0] != severity {
		return false
	}
	return line[1] >= '0' && line[1] <= '9'
}

func incrementMetric(metrics map[string]any, key string) {
	count, _ := metrics[key].(int)
	metrics[key] = count + 1
}

func appendLimited(values []string, value string, limit int) []string {
	values = append(values, value)
	if len(values) <= limit {
		return values
	}
	return values[len(values)-limit:]
}

func statusPayloadBySuffix(probes []ProbeOutputSummary, suffix string) (map[string]any, string) {
	for _, probe := range probes {
		samples, ok := probe.Payload["samples"].(map[string]any)
		if !ok {
			continue
		}
		for topic, rawSample := range samples {
			if !strings.HasSuffix(topic, suffix) {
				continue
			}
			sample, ok := rawSample.(map[string]any)
			if !ok {
				continue
			}
			if parsed, ok := sample["parsed"].(map[string]any); ok {
				return parsed, topic
			}
			data, ok := sample["data"].(string)
			if !ok || strings.TrimSpace(data) == "" {
				continue
			}
			payload := map[string]any{}
			if err := json.Unmarshal([]byte(data), &payload); err != nil {
				continue
			}
			return payload, topic
		}
	}
	return nil, ""
}

func probePayloadByName(probes []ProbeOutputSummary, name string) (map[string]any, string) {
	for _, probe := range probes {
		if probe.Name != name || len(probe.Payload) == 0 {
			continue
		}
		return probe.Payload, probe.Name
	}
	return nil, ""
}

func probeBlockers(probe ProbeOutputSummary) []string {
	blockers := stringListFromAny(probe.Payload["blockers"])
	samples, ok := probe.Payload["samples"].(map[string]any)
	if !ok {
		return blockers
	}
	for topic, rawSample := range samples {
		if strings.HasSuffix(topic, "/landing/status") {
			continue
		}
		sample, ok := rawSample.(map[string]any)
		if !ok {
			continue
		}
		if parsed, ok := sample["parsed"].(map[string]any); ok {
			blockers = append(blockers, stringListFromAny(parsed["blockers"])...)
		}
	}
	return blockers
}

func stringListFromAny(raw any) []string {
	values, ok := raw.([]any)
	if !ok {
		return nil
	}
	out := make([]string, 0, len(values))
	for _, value := range values {
		text, ok := value.(string)
		if !ok || strings.TrimSpace(text) == "" {
			continue
		}
		out = append(out, text)
	}
	return out
}

func stringAnySliceContains(raw any, expected string) bool {
	for _, value := range stringListFromAny(raw) {
		if value == expected {
			return true
		}
	}
	return false
}

func subsetMap(source map[string]any, keys ...string) map[string]any {
	result := map[string]any{}
	for _, key := range keys {
		if value, ok := source[key]; ok {
			result[key] = value
		}
	}
	if len(result) == 0 {
		return nil
	}
	return result
}

func landingFromProbeOutputs(probes []ProbeOutputSummary) *helpers.Landing {
	var fallback *helpers.Landing
	for _, probe := range probes {
		samples, ok := probe.Payload["samples"].(map[string]any)
		if !ok {
			continue
		}
		for topic, rawSample := range samples {
			if !strings.HasSuffix(topic, "/landing/status") {
				continue
			}
			sample, ok := rawSample.(map[string]any)
			if !ok {
				continue
			}
			payload := map[string]any{}
			if parsed, ok := sample["parsed"].(map[string]any); ok {
				payload = parsed
			} else if data, ok := sample["data"].(string); ok && strings.TrimSpace(data) != "" {
				if err := json.Unmarshal([]byte(data), &payload); err != nil {
					continue
				}
			}
			if len(payload) == 0 {
				continue
			}
			data, err := json.Marshal(payload)
			if err != nil {
				continue
			}
			var landing helpers.Landing
			if err := json.Unmarshal(data, &landing); err != nil {
				continue
			}
			if landing.OK {
				return &landing
			}
			value := landing
			fallback = &value
		}
	}
	return fallback
}

func evaluateProbeOutputs(plan Plan, artifactDir string) []ProbeOutputSummary {
	summaries := make([]ProbeOutputSummary, 0, len(plan.Execution.ROSProbes))
	for _, probe := range plan.Execution.ROSProbes {
		path := filepath.Join(artifactDir, probe.OutputPath)
		summary := ProbeOutputSummary{Name: probe.Name, Path: path}
		data, err := readEventuallyStableProbeOutput(path)
		if err != nil {
			summaries = append(summaries, summary)
			continue
		}
		summary.Exists = true
		payload := map[string]any{}
		if err := json.Unmarshal(data, &payload); err != nil {
			summary.OK = strings.TrimSpace(string(data)) != ""
			summaries = append(summaries, summary)
			continue
		}
		summary.Payload = payload
		if status, _ := payload["status"].(string); status != "" {
			summary.Status = status
		}
		if ok, exists := payload["ok"].(bool); exists {
			summary.OK = ok
		} else {
			summary.OK = len(payload) > 0
		}
		summaries = append(summaries, summary)
	}
	return summaries
}

func readEventuallyStableProbeOutput(path string) ([]byte, error) {
	var lastData []byte
	var lastErr error
	for attempt := 0; attempt < 50; attempt++ {
		data, err := os.ReadFile(path)
		if err != nil {
			lastErr = err
			time.Sleep(100 * time.Millisecond)
			continue
		}
		lastData = data
		if len(strings.TrimSpace(string(data))) == 0 {
			time.Sleep(100 * time.Millisecond)
			continue
		}
		payload := map[string]any{}
		if err := json.Unmarshal(data, &payload); err == nil {
			return data, nil
		}
		time.Sleep(100 * time.Millisecond)
	}
	if lastData != nil {
		return lastData, nil
	}
	return nil, lastErr
}

func evaluateRosbagProfiles(plan Plan, artifactDir string) []RosbagGateSummary {
	summaries := make([]RosbagGateSummary, 0, len(plan.Execution.RosbagRecords))
	for _, rosbag := range plan.Execution.RosbagRecords {
		requiredTopics := rosbag.RequiredTopics
		if len(requiredTopics) == 0 {
			requiredTopics = rosbag.Topics
		}
		metadataPath := filepath.Join(artifactDir, rosbag.OutputDir, "metadata.yaml")
		summary := RosbagGateSummary{
			Name:           rosbag.Name,
			MetadataPath:   metadataPath,
			RequiredTopics: append([]string(nil), requiredTopics...),
		}
		data, err := os.ReadFile(metadataPath)
		if err != nil {
			summary.MetadataMissingOrUnreadable = err.Error()
			summaries = append(summaries, summary)
			continue
		}
		summary.Exists = true
		counts := helpers.MetadataCountsFromString(string(data))
		summary.MessageCounts = counts
		for _, topic := range requiredTopics {
			count, exists := counts[topic]
			if !exists {
				summary.MissingRequiredTopics = append(summary.MissingRequiredTopics, topic)
				continue
			}
			if count <= 0 {
				summary.ZeroCountRequiredTopics = append(summary.ZeroCountRequiredTopics, topic)
			}
		}
		summary.OK = len(summary.MissingRequiredTopics) == 0 && len(summary.ZeroCountRequiredTopics) == 0
		summaries = append(summaries, summary)
	}
	return summaries
}

func taskSpecificChecks(runtimeConfig config.TaskRuntimeConfig, taskID string) map[string]bool {
	checks := map[string]bool{}
	switch taskID {
	case "hover":
		checks["hover_takeoff_altitude_positive"] = runtimeConfig.FCUController.TakeoffAltM > 0
		checks["hover_landing_policy_land_in_place"] = runtimeConfig.Landing.HoverPolicy == "" || runtimeConfig.Landing.HoverPolicy == helpers.PolicyLandInPlace
		checks["hover_claim_evaluated"] = runtimeConfig.SlamHover.HoverClaim == "evaluated"
	case "exploration":
		checks["exploration_window_positive"] = runtimeConfig.ExplorationGate.ExplorationWindowSec > 0
		checks["exploration_min_goals_positive"] = runtimeConfig.ExplorationGate.MinAcceptedGoals > 0
		checks["exploration_landing_policy_valid"] = validLandingPolicy(runtimeConfig.Landing.ExplorationPolicy)
		checks["exploration_claim_evaluated"] = runtimeConfig.ExplorationGate.ExplorationClaim == "evaluated"
	case "navigation":
		checks["nav2_enabled"] = runtimeConfig.Nav2.Enabled
		checks["nav2_frames_configured"] = runtimeConfig.Nav2.GlobalFrame != "" && runtimeConfig.Nav2.OdomFrame != "" && runtimeConfig.Nav2.BaseFrame != ""
		checks["nav2_costmap_layers_configured"] = len(runtimeConfig.Nav2.Costmap.RequiredLayers) > 0
		checks["navigation_adapter_limits_positive"] = runtimeConfig.NavigationAdapter.MaxXYSpeedMPS > 0 && runtimeConfig.NavigationAdapter.FixedAltitudeM > 0
		checks["navigation_mission_positive"] = runtimeConfig.NavigationMission.NavigationWindowSec > 0 && runtimeConfig.NavigationMission.MinAcceptedGoals > 0 && runtimeConfig.NavigationMission.MinCoverageGrowth > 0
		checks["navigation_landing_policy_valid"] = validLandingPolicy(runtimeConfig.Landing.NavigationPolicy)
		checks["navigation_exit_goal_configured"] = runtimeConfig.NavigationMission.ExitGoal.ID != ""
		checks["navigation_claim_evaluated"] = runtimeConfig.NavigationMission.NavigationClaim == "evaluated"
	case "scan-robustness":
		scanStabilizationBlockers := helpers.ValidateScanStabilizationConfig(scanStabilizationSpec(runtimeConfig))
		checks["scan_stabilization_config_valid"] = len(scanStabilizationBlockers) == 0
		checks["airframe_required_profiles_configured"] = len(runtimeConfig.AirframeDisturbanceGate.RequiredProfiles) > 0
		checks["airframe_profile_configured"] = runtimeConfig.AirframeDisturbance.Profile != ""
		checks["scan_robustness_landing_policy_land_in_place"] = runtimeConfig.Landing.ScanRobustnessPolicy == "" || runtimeConfig.Landing.ScanRobustnessPolicy == helpers.PolicyLandInPlace
	}
	return checks
}

func taskSpecificBlockers(runtimeConfig config.TaskRuntimeConfig, taskID string) []string {
	switch taskID {
	case "navigation":
		return validateNavigationConfig(runtimeConfig)
	default:
		return nil
	}
}

func validateNavigationConfig(runtimeConfig config.TaskRuntimeConfig) []string {
	blockers := []string{}
	nav2 := runtimeConfig.Nav2
	costmap := runtimeConfig.Nav2.Costmap
	adapter := runtimeConfig.NavigationAdapter
	mission := runtimeConfig.NavigationMission
	if !nav2.Enabled {
		blockers = append(blockers, "nav2_config_disabled")
	}
	if nav2.GlobalFrame == "" || nav2.OdomFrame == "" || nav2.BaseFrame == "" {
		blockers = append(blockers, "nav2_tf_invalid:frame_missing")
	}
	if nav2.ScanTopic == "" || nav2.MapTopic == "" || nav2.CmdVelTopic == "" {
		blockers = append(blockers, "navigation_topic_missing")
	}
	if runtimeConfig.ScanStabilization.OutputScanTopic != "" && nav2.ScanTopic != runtimeConfig.ScanStabilization.OutputScanTopic {
		blockers = append(blockers, "navigation_scan_source_not_stabilized")
	}
	if nav2.CmdVelTopic == runtimeConfig.FCUController.CmdVelTopic || nav2.CmdVelTopic == "/ap/v1/cmd_vel" {
		blockers = append(blockers, "nav2_direct_fcu_cmd_vel_publisher")
	}
	if adapter.SetpointIntentTopic == runtimeConfig.FCUController.CmdVelTopic || adapter.SetpointIntentTopic == "/ap/v1/cmd_vel" {
		blockers = append(blockers, "navigation_adapter_direct_fcu_cmd_vel_publisher")
	}
	if !stringSliceContains(costmap.RequiredLayers, "obstacle_layer") {
		blockers = append(blockers, "nav2_obstacle_layer_missing")
	}
	if !stringSliceContains(costmap.RequiredLayers, "inflation_layer") {
		blockers = append(blockers, "nav2_inflation_layer_missing")
	}
	if costmap.GlobalCostmapTopic == "" || costmap.LocalCostmapTopic == "" {
		blockers = append(blockers, "navigation_costmap_topic_missing")
	}
	if costmap.MaxCostmapAgeSec <= 0 {
		blockers = append(blockers, "nav2_costmap_stale_threshold_invalid")
	}
	if costmap.MaxUnknownRatio <= 0 || costmap.MaxUnknownRatio > 1 {
		blockers = append(blockers, "navigation_costmap_unknown_ratio_invalid")
	}
	if costmap.UsesGazeboTruth || mission.UsesGazeboTruthAsInput {
		blockers = append(blockers, "navigation_uses_gazebo_truth_as_input")
	}
	if adapter.MaxXYSpeedMPS <= 0 || adapter.MaxYawRateDPS <= 0 || adapter.MaxAccelMPS2 <= 0 {
		blockers = append(blockers, "navigation_adapter_limit_invalid")
	}
	if adapter.FixedAltitudeM <= 0 {
		blockers = append(blockers, "navigation_fixed_altitude_invalid")
	}
	if mission.NavigationWindowSec <= 0 || mission.MinAcceptedGoals <= 0 || mission.MinPathLengthM <= 0 {
		blockers = append(blockers, "navigation_mission_threshold_invalid")
	}
	if mission.MinCoverageGrowth <= 0 || mission.MinCoverageGrowth > 1 {
		blockers = append(blockers, "navigation_coverage_threshold_invalid")
	}
	if mission.GoalFrame == "" || (mission.ExitGoal.ID == "" && len(mission.BoundedGoals) == 0) {
		blockers = append(blockers, "navigation_goal_contract_missing")
	}
	if mission.ExitGoal.ID == "" {
		blockers = append(blockers, "navigation_exit_goal_missing")
	}
	if mission.CompletionPolicy != "" && !validLandingPolicy(mission.CompletionPolicy) {
		blockers = append(blockers, "navigation_completion_policy_invalid")
	}
	return blockers
}

func validLandingPolicy(policy string) bool {
	return policy == "" || policy == helpers.PolicyLandInPlace || policy == helpers.PolicyReturnHomeThenLand
}

func landingConfig(project config.ProjectConfig, runtimeConfig config.TaskRuntimeConfig, taskID string) helpers.Config {
	landing := runtimeConfig.Landing
	policy := landing.DefaultPolicy
	switch taskID {
	case "hover":
		policy = landing.HoverPolicy
	case "exploration":
		policy = landing.ExplorationPolicy
	case "navigation":
		policy = landing.NavigationPolicy
	case "scan-robustness":
		policy = landing.ScanRobustnessPolicy
	}
	return helpers.Config{
		Enabled:                   landing.Enabled,
		Policy:                    policy,
		DefaultPolicy:             landing.DefaultPolicy,
		LandingStatusTopic:        landing.LandingStatusTopic,
		LandingIntentTopic:        landing.LandingIntentTopic,
		HomeSource:                landing.HomeSource,
		HomeRadiusM:               landing.HomeRadiusM,
		PreLandHoldSec:            landing.PreLandHoldSec,
		CompletionGraceSec:        landing.CompletionGraceSec,
		MaxReturnHomeDurationSec:  landing.MaxReturnHomeDurationSec,
		MaxLandingDurationSec:     landing.MaxLandingDurationSec,
		MaxDescentRateMPS:         landing.MaxDescentRateMPS,
		TouchdownAltitudeM:        landing.TouchdownAltitudeM,
		TouchdownVerticalSpeedMPS: landing.TouchdownVerticalSpeedMPS,
		RequireDisarm:             landing.RequireDisarm,
		RequireMotorsSafe:         landing.RequireMotorsSafe,
		UsesGazeboTruthAsInput:    project.Landing.UsesGazeboTruthAsInput,
	}
}

func stringSliceContains(values []string, target string) bool {
	for _, value := range values {
		if value == target {
			return true
		}
	}
	return false
}

func uniqueStrings(values []string) []string {
	seen := map[string]bool{}
	for _, value := range values {
		if strings.TrimSpace(value) == "" {
			continue
		}
		seen[value] = true
	}
	out := make([]string, 0, len(seen))
	for value := range seen {
		out = append(out, value)
	}
	sort.Strings(out)
	return out
}
