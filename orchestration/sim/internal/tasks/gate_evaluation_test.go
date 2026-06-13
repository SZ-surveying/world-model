package tasks

import (
	"os"
	"path/filepath"
	"testing"

	"navlab/orchestration-sim/internal/config"
	"navlab/orchestration-sim/internal/tasks/helpers"
)

func TestEvaluateResultGatesReadsProbeAndRosbagArtifacts(t *testing.T) {
	artifactDir := t.TempDir()
	if err := os.WriteFile(filepath.Join(artifactDir, "hover_probe.json"), []byte(`{"ok":true,"status":"live_probe"}`+"\n"), 0o644); err != nil {
		t.Fatal(err)
	}
	rosbagDir := filepath.Join(artifactDir, "rosbag")
	if err := os.MkdirAll(rosbagDir, 0o755); err != nil {
		t.Fatal(err)
	}
	if err := os.WriteFile(filepath.Join(rosbagDir, "metadata.yaml"), []byte("topics_with_message_count:\n- topic_metadata:\n    name: /slam/odom\n  message_count: 3\n"), 0o644); err != nil {
		t.Fatal(err)
	}

	evaluation := EvaluateResultGates(
		config.ProjectConfig{},
		config.TaskRuntimeConfig{
			Landing:       config.LandingConfig{HoverPolicy: "land_in_place", DefaultPolicy: "land_in_place"},
			FCUController: config.FCUControllerConfig{TakeoffAltM: 0.5},
			SlamHover:     config.SlamHoverConfig{HoverClaim: "evaluated"},
		},
		Plan{
			TaskID: "hover",
			Execution: helpers.ExecutionPlan{
				ROSProbes: []helpers.ROSProbePlan{{Name: "hover_probe", OutputPath: "hover_probe.json"}},
				RosbagRecords: []helpers.RosbagRecordPlan{
					{Name: "hover_rosbag", OutputDir: "rosbag", Topics: []string{"/slam/odom"}},
				},
			},
		},
		artifactDir,
		RuntimeSpecBundle{},
		RuntimeExecutionResult{},
		nil,
	)
	if evaluation.OK || !evaluation.Blocked {
		t.Fatalf("evaluation status = ok:%v blocked:%v", evaluation.OK, evaluation.Blocked)
	}
	if len(evaluation.ProbeOutputs) != 1 || !evaluation.ProbeOutputs[0].OK {
		t.Fatalf("probe outputs = %#v", evaluation.ProbeOutputs)
	}
	if len(evaluation.RosbagProfiles) != 1 || !evaluation.RosbagProfiles[0].OK {
		t.Fatalf("rosbag profiles = %#v", evaluation.RosbagProfiles)
	}
	if !blockersContainPrefix(evaluation.Blockers, helpers.LandingNotEvaluatedBlocker) {
		t.Fatalf("blockers = %#v", evaluation.Blockers)
	}
}

func TestMetricSummaryIncludesP12DeepMetrics(t *testing.T) {
	metrics := metricSummaryFromEvidence([]ProbeOutputSummary{
		{
			Name: "p12_probe",
			Payload: map[string]any{
				"samples": map[string]any{
					"/navlab/airframe_disturbance/status": map[string]any{
						"data": `{"claim":"evaluated","profile":"realistic","estimated_attitude_response_lag_ms":12.5,"attitude_overshoot_count":1,"attitude_noise_rms_deg":0.4,"false_drop_ratio":0.02,"compensation_jitter_score":0.1,"scan_integrity":{"scan_contract":"p11_stabilized_scan","false_drop_ratio":0.02}}`,
					},
					"/navlab/scan_integrity/status": map[string]any{
						"data": `{"claim":"evaluated","scan_contract":"p11_stabilized_scan","scan_samples":42,"drop_ratio":0.01,"false_drop_ratio":0.02,"compensated_ratio":0.3,"floor_hit_risk_beam_ratio":0.0,"max_scan_attitude_time_offset_ms":4.0}`,
					},
				},
			},
		},
	}, nil)
	if metrics.AirframeDisturbance["estimated_attitude_response_lag_ms"] == nil ||
		metrics.AirframeDisturbance["attitude_overshoot_count"] == nil ||
		metrics.AirframeDisturbance["attitude_noise_rms_deg"] == nil ||
		metrics.AirframeDisturbance["false_drop_ratio"] == nil ||
		metrics.AirframeDisturbance["compensation_jitter_score"] == nil ||
		metrics.AirframeDisturbance["scan_integrity"] == nil {
		t.Fatalf("airframe metrics = %#v", metrics.AirframeDisturbance)
	}
	if metrics.ScanIntegrity["scan_contract"] != "p11_stabilized_scan" || metrics.ScanIntegrity["false_drop_ratio"] == nil {
		t.Fatalf("scan integrity metrics = %#v", metrics.ScanIntegrity)
	}
}
