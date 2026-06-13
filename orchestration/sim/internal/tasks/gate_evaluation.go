package tasks

import (
	"encoding/json"
	"fmt"
	"os"
	"path/filepath"
	"sort"
	"strings"

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
	Controller            map[string]any            `json:"controller,omitempty"`
	Owner                 map[string]any            `json:"owner,omitempty"`
	Exploration           map[string]any            `json:"exploration,omitempty"`
	SLAM                  map[string]any            `json:"slam,omitempty"`
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
	metrics := metricSummaryFromEvidence(probeOutputs, rosbagProfiles)

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

func metricSummaryFromEvidence(probes []ProbeOutputSummary, rosbags []RosbagGateSummary) MetricSummary {
	summary := MetricSummary{
		MetricEvidenceSources: map[string]string{},
	}
	if payload, topic := statusPayloadBySuffix(probes, "/hover/status"); payload != nil {
		summary.Hover = subsetMap(payload, "claim", "state", "pose_samples", "max_hover_horizontal_drift_m", "max_hover_altitude_error_m", "max_hover_yaw_drift_rad", "drift_reference")
		summary.MetricEvidenceSources["hover"] = topic
	}
	if payload, topic := statusPayloadBySuffix(probes, "/fcu/controller/status"); payload != nil {
		summary.Controller = subsetMap(payload, "ready", "state", "pose_samples", "control_route", "takeoff_alt_m", "fcu_mode_window")
		summary.MetricEvidenceSources["controller"] = topic
	}
	if payload, topic := statusPayloadBySuffix(probes, "/fcu/owner/status"); payload != nil {
		summary.Owner = subsetMap(payload, "active", "owner", "active_owner_count", "owner_unique", "conflicting_owners")
		summary.MetricEvidenceSources["owner"] = topic
	}
	if payload, topic := statusPayloadBySuffix(probes, "/exploration/status"); payload != nil {
		summary.Exploration = subsetMap(payload, "claim", "strategy", "accepted_goals", "min_accepted_goals", "path_length_m", "min_path_length_m", "motion_speed_mps")
		summary.MetricEvidenceSources["exploration"] = topic
	}
	if payload, topic := statusPayloadBySuffix(probes, "/slam/status"); payload != nil {
		summary.SLAM = subsetMap(payload, "ready", "tracking_state", "odom_samples", "pose_samples", "max_position_jump_m", "map_frame", "base_frame", "quality")
		summary.MetricEvidenceSources["slam"] = topic
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
			data, ok := sample["data"].(string)
			if !ok || strings.TrimSpace(data) == "" {
				continue
			}
			var landing helpers.Landing
			if err := json.Unmarshal([]byte(data), &landing); err != nil {
				continue
			}
			return &landing
		}
	}
	return nil
}

func evaluateProbeOutputs(plan Plan, artifactDir string) []ProbeOutputSummary {
	summaries := make([]ProbeOutputSummary, 0, len(plan.Execution.ROSProbes))
	for _, probe := range plan.Execution.ROSProbes {
		path := filepath.Join(artifactDir, probe.OutputPath)
		summary := ProbeOutputSummary{Name: probe.Name, Path: path}
		data, err := os.ReadFile(path)
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

func evaluateRosbagProfiles(plan Plan, artifactDir string) []RosbagGateSummary {
	summaries := make([]RosbagGateSummary, 0, len(plan.Execution.RosbagRecords))
	for _, rosbag := range plan.Execution.RosbagRecords {
		metadataPath := filepath.Join(artifactDir, rosbag.OutputDir, "metadata.yaml")
		summary := RosbagGateSummary{
			Name:           rosbag.Name,
			MetadataPath:   metadataPath,
			RequiredTopics: append([]string(nil), rosbag.Topics...),
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
		for _, topic := range rosbag.Topics {
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
		checks["exploration_landing_policy_return_home"] = runtimeConfig.Landing.ExplorationPolicy == helpers.PolicyReturnHomeThenLand
		checks["exploration_claim_evaluated"] = runtimeConfig.ExplorationGate.ExplorationClaim == "evaluated"
	case "scan-robustness":
		scanStabilizationBlockers := helpers.ValidateScanStabilizationConfig(scanStabilizationSpec(runtimeConfig))
		checks["scan_stabilization_config_valid"] = len(scanStabilizationBlockers) == 0
		checks["airframe_required_profiles_configured"] = len(runtimeConfig.AirframeDisturbanceGate.RequiredProfiles) > 0
		checks["airframe_profile_configured"] = runtimeConfig.AirframeDisturbance.Profile != ""
		checks["scan_robustness_landing_policy_land_in_place"] = runtimeConfig.Landing.ScanRobustnessPolicy == "" || runtimeConfig.Landing.ScanRobustnessPolicy == helpers.PolicyLandInPlace
	}
	return checks
}

func landingConfig(project config.ProjectConfig, runtimeConfig config.TaskRuntimeConfig, taskID string) helpers.Config {
	landing := runtimeConfig.Landing
	policy := landing.DefaultPolicy
	switch taskID {
	case "hover":
		policy = landing.HoverPolicy
	case "exploration":
		policy = landing.ExplorationPolicy
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
