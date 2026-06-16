package tasks

import (
	"os"
	"path/filepath"
	"strings"
	"testing"
	"time"

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
	if !stringSliceContains(evaluation.Blockers, "hover_mission_summary_missing") {
		t.Fatalf("blockers = %#v, want hover_mission_summary_missing", evaluation.Blockers)
	}
}

func TestEvaluateProbeOutputsRetriesUntilJSONPayloadIsReadable(t *testing.T) {
	artifactDir := t.TempDir()
	probePath := filepath.Join(artifactDir, "slam_hover_probe.json")
	if err := os.WriteFile(probePath, nil, 0o644); err != nil {
		t.Fatal(err)
	}
	go func() {
		time.Sleep(1200 * time.Millisecond)
		_ = os.WriteFile(probePath, []byte(`{"ok":false,"status":"sampled","samples":{"/external_nav/status":{"parsed":{"ready":false,"state":"low_rate","odom":{"input_topic":"/slam/odom","rate_ok":false}}}}}`), 0o644)
	}()

	outputs := evaluateProbeOutputs(Plan{
		Execution: helpers.ExecutionPlan{
			ROSProbes: []helpers.ROSProbePlan{{Name: "slam_hover_probe", OutputPath: "slam_hover_probe.json"}},
		},
	}, artifactDir)

	if len(outputs) != 1 || !outputs[0].Exists || outputs[0].OK || len(outputs[0].Payload) == 0 {
		t.Fatalf("probe outputs = %#v", outputs)
	}
	metrics := metricSummaryFromEvidence(outputs, nil)
	if metrics.ExternalNav["state"] != "low_rate" {
		t.Fatalf("external nav metrics = %#v", metrics.ExternalNav)
	}
}

func TestHoverMissionBlockersRequirePythonMissionSummary(t *testing.T) {
	if blockers := hoverMissionBlockers(nil); !stringSliceContains(blockers, "hover_mission_summary_missing") {
		t.Fatalf("blockers = %#v, want hover_mission_summary_missing", blockers)
	}
	valid := map[string]any{
		"ok":                         true,
		"hover_body_ok":              true,
		"landing_ok":                 true,
		"phases_seen":                []any{"wait_ready", "guided", "arm", "takeoff", "hover_settle", "hover_hold", "landing", "complete"},
		"guided_seen":                true,
		"armed_seen":                 true,
		"takeoff_ack_ok":             false,
		"airborne_seen":              true,
		"local_position_count":       float64(12),
		"setpoints_sent_count":       float64(40),
		"target_alt_m":               float64(0.5),
		"takeoff_alt_m":              float64(0.5),
		"current_z_ned":              float64(-0.45),
		"altitude_error_m":           float64(0.05),
		"hover_altitude_tolerance_m": float64(0.18),
		"hover_hold_sec":             float64(18),
		"hover_drift": map[string]any{
			"sample_count":           float64(20),
			"duration_sec":           float64(17.9),
			"duration_tolerance_sec": float64(0.25),
			"horizontal_drift_m":     float64(0.05),
			"z_span_m":               float64(0.02),
			"max_horizontal_drift_m": float64(0.35),
			"max_altitude_drift_m":   float64(0.30),
			"ok":                     true,
		},
		"landing": map[string]any{
			"land_command_accepted": true,
			"touchdown_confirmed":   true,
			"disarmed":              true,
			"motors_safe":           true,
		},
	}
	if blockers := hoverMissionBlockers(valid); len(blockers) != 0 {
		t.Fatalf("blockers = %#v, want none", blockers)
	}
	tooLow := map[string]any{
		"ok":                         false,
		"hover_body_ok":              false,
		"landing_ok":                 false,
		"phases_seen":                []any{"wait_ready", "guided", "arm", "takeoff", "hover_settle"},
		"guided_seen":                true,
		"armed_seen":                 true,
		"takeoff_ack_ok":             false,
		"airborne_seen":              true,
		"local_position_count":       float64(5),
		"setpoints_sent_count":       float64(10),
		"target_alt_m":               float64(0.5),
		"takeoff_alt_m":              float64(0.5),
		"current_z_ned":              float64(-0.19),
		"altitude_error_m":           float64(0.31),
		"hover_altitude_tolerance_m": float64(0.18),
		"hover_hold_sec":             float64(18),
		"hover_drift": map[string]any{
			"sample_count":           float64(1),
			"duration_sec":           float64(2.0),
			"duration_tolerance_sec": float64(0.25),
			"ok":                     false,
		},
		"landing": map[string]any{
			"land_command_accepted": false,
			"touchdown_confirmed":   false,
			"disarmed":              false,
			"motors_safe":           false,
		},
	}
	blockers := hoverMissionBlockers(tooLow)
	for _, want := range []string{
		"hover_mission_not_ok",
		"hover_mission_hover_hold_missing",
		"hover_mission_takeoff_ack_missing",
		"hover_mission_target_altitude_not_reached",
		"hover_mission_hold_samples_missing",
		"hover_mission_hold_duration_short",
		"hover_mission_drift_not_ok",
		"hover_mission_body_not_ok",
		"hover_mission_landing_not_ok",
		"hover_mission_landing_touchdown_confirmed_missing",
		"hover_mission_landing_disarmed_missing",
		"hover_mission_landing_motors_safe_missing",
	} {
		if !stringSliceContains(blockers, want) {
			t.Fatalf("blockers = %#v, want %s", blockers, want)
		}
	}
}

func TestHoverMissionBlockersRejectMissingAltitudeEvidence(t *testing.T) {
	mission := map[string]any{
		"ok":                   true,
		"hover_body_ok":        true,
		"landing_ok":           true,
		"phases_seen":          []any{"hover_hold"},
		"guided_seen":          true,
		"armed_seen":           true,
		"takeoff_ack_ok":       true,
		"airborne_seen":        true,
		"local_position_count": float64(1),
		"setpoints_sent_count": float64(1),
		"target_alt_m":         float64(0.5),
		"hover_hold_sec":       float64(18),
		"hover_drift": map[string]any{
			"sample_count":           float64(2),
			"duration_sec":           float64(18),
			"duration_tolerance_sec": float64(0.25),
			"ok":                     true,
		},
		"landing": map[string]any{
			"land_command_accepted": true,
			"touchdown_confirmed":   true,
			"disarmed":              true,
			"motors_safe":           true,
		},
	}
	blockers := hoverMissionBlockers(mission)
	if !stringSliceContains(blockers, "hover_mission_altitude_evidence_missing") {
		t.Fatalf("blockers = %#v, want hover_mission_altitude_evidence_missing", blockers)
	}
}

func TestExternalNavFeedbackBlockersRequireSlamBridgeAndFCUFeedback(t *testing.T) {
	externalNav := map[string]any{
		"ready": true,
		"odom":  map[string]any{"input_topic": "/slam/odom"},
	}
	mavlinkExternalNav := map[string]any{
		"state":                     "sending",
		"ready":                     true,
		"input_topic":               "/external_nav/odom",
		"sent_count":                float64(12),
		"local_position_count":      float64(5),
		"local_position_age_ms":     float64(50),
		"max_local_position_age_ms": float64(1000),
		"fcu_local_position_ready":  true,
	}
	if blockers := externalNavFeedbackBlockers(externalNav, mavlinkExternalNav); len(blockers) != 0 {
		t.Fatalf("blockers = %#v, want none", blockers)
	}

	mavlinkExternalNav["local_position_count"] = float64(0)
	mavlinkExternalNav["fcu_local_position_ready"] = false
	blockers := externalNavFeedbackBlockers(externalNav, mavlinkExternalNav)
	if !stringSliceContains(blockers, "external_nav_not_seen_by_fcu") {
		t.Fatalf("blockers = %#v, want external_nav_not_seen_by_fcu", blockers)
	}
}

func TestExternalNavFeedbackBlockersRejectStaleFCULocalPosition(t *testing.T) {
	blockers := externalNavFeedbackBlockers(
		map[string]any{
			"ready": true,
			"odom":  map[string]any{"input_topic": "/slam/odom"},
		},
		map[string]any{
			"state":                     "sending",
			"ready":                     false,
			"input_topic":               "/external_nav/odom",
			"sent_count":                float64(12),
			"local_position_count":      float64(554),
			"local_position_age_ms":     float64(58650.282),
			"max_local_position_age_ms": float64(1000),
			"fcu_local_position_ready":  false,
		},
	)
	for _, want := range []string{"mavlink_external_nav_not_ready", "external_nav_fcu_local_position_not_ready", "external_nav_fcu_local_position_stale"} {
		if !stringSliceContains(blockers, want) {
			t.Fatalf("blockers = %#v, want %s", blockers, want)
		}
	}
}

func TestExternalNavFeedbackBlockersRejectDiagnosticTruthInput(t *testing.T) {
	blockers := externalNavFeedbackBlockers(
		map[string]any{
			"ready": true,
			"odom":  map[string]any{"input_topic": "/odometry"},
		},
		map[string]any{
			"state":                    "sending",
			"ready":                    true,
			"input_topic":              "/external_nav/odom",
			"sent_count":               float64(12),
			"local_position_count":     float64(5),
			"local_position_age_ms":    float64(50),
			"fcu_local_position_ready": true,
		},
	)
	for _, want := range []string{"external_nav_not_using_slam_odom", "external_nav_uses_diagnostic_truth_input"} {
		if !stringSliceContains(blockers, want) {
			t.Fatalf("blockers = %#v, want %s", blockers, want)
		}
	}
}

func TestLandingEvidenceSupportsParsedProbeSample(t *testing.T) {
	landing := landingFromProbeOutputs([]ProbeOutputSummary{
		{
			Payload: map[string]any{
				"samples": map[string]any{
					"/navlab/landing/status": map[string]any{
						"parsed": map[string]any{
							"ok":       false,
							"claim":    helpers.ClaimNotEvaluated,
							"policy":   helpers.PolicyLandInPlace,
							"state":    "waiting_for_task_completion",
							"blockers": []any{"task_completion_required_before_landing"},
						},
					},
				},
			},
		},
		{
			Payload: map[string]any{
				"samples": map[string]any{
					"/navlab/landing/status": map[string]any{
						"parsed": map[string]any{
							"ok":                    true,
							"claim":                 helpers.ClaimEvaluated,
							"policy":                helpers.PolicyLandInPlace,
							"state":                 "landing_complete",
							"task_completed":        true,
							"land_command_accepted": true,
							"landed_confirmed":      true,
							"touchdown_confirmed":   true,
							"disarmed":              true,
							"motors_safe":           true,
							"blockers":              []any{},
						},
					},
				},
			},
		},
	})
	if landing == nil || !landing.OK || landing.Claim != helpers.ClaimEvaluated {
		t.Fatalf("landing = %#v", landing)
	}
}

func TestEvaluateResultGatesUsesLandingStatusFromNavigationProbe(t *testing.T) {
	artifactDir := t.TempDir()
	probe := `{
		"ok": true,
		"status": "sampled",
		"samples": {
			"/navlab/landing/status": {
				"parsed": {
					"ok": true,
					"claim": "evaluated",
					"policy": "land_in_place",
					"state": "landing_complete",
					"task_completed": true,
					"land_command_accepted": true,
					"landed_confirmed": true,
					"touchdown_confirmed": true,
					"disarmed": true,
					"motors_safe": true,
					"blockers": []
				}
			}
		}
	}`
	if err := os.WriteFile(filepath.Join(artifactDir, "navigation_status_probe.json"), []byte(probe+"\n"), 0o644); err != nil {
		t.Fatal(err)
	}
	earlyProbe := `{
		"ok": true,
		"status": "sampled",
		"samples": {
			"/navlab/landing/status": {
				"parsed": {
					"ok": false,
					"claim": "not_evaluated",
					"policy": "land_in_place",
					"state": "waiting_for_task_completion",
					"blockers": ["task_completion_required_before_landing"]
				}
			}
		}
	}`
	if err := os.WriteFile(filepath.Join(artifactDir, "slam_hover_probe.json"), []byte(earlyProbe+"\n"), 0o644); err != nil {
		t.Fatal(err)
	}
	evaluation := EvaluateResultGates(
		config.ProjectConfig{},
		config.TaskRuntimeConfig{
			Landing: config.LandingConfig{
				Enabled:       true,
				DefaultPolicy: helpers.PolicyLandInPlace,
			},
		},
		Plan{
			TaskID: "navigation",
			Execution: helpers.ExecutionPlan{
				ROSProbes: []helpers.ROSProbePlan{
					{Name: "slam_hover_probe", OutputPath: "slam_hover_probe.json"},
					{Name: "navigation_status_probe", OutputPath: "navigation_status_probe.json"},
				},
			},
		},
		artifactDir,
		RuntimeSpecBundle{},
		RuntimeExecutionResult{},
		nil,
	)
	if !evaluation.Landing.OK || evaluation.Landing.LandingClaim != helpers.ClaimEvaluated {
		t.Fatalf("landing acceptance = %#v", evaluation.Landing)
	}
	if blockersContainPrefix(evaluation.Blockers, "task_completion_required_before_landing") {
		t.Fatalf("blockers = %#v", evaluation.Blockers)
	}
}

func TestNavigationConfigBlockersCatchUnsafeRoutesAndMissingContracts(t *testing.T) {
	runtimeConfig := validNavigationRuntimeConfig()
	runtimeConfig.Nav2.GlobalFrame = ""
	runtimeConfig.Nav2.ScanTopic = "/navlab/x2/scan_raw"
	runtimeConfig.Nav2.CmdVelTopic = runtimeConfig.FCUController.CmdVelTopic
	runtimeConfig.Nav2.Costmap.RequiredLayers = []string{"static_layer"}
	runtimeConfig.Nav2.Costmap.UsesGazeboTruth = true
	runtimeConfig.Nav2.Costmap.GlobalCostmapTopic = ""
	runtimeConfig.Nav2.Costmap.MaxCostmapAgeSec = 0
	runtimeConfig.Nav2.Costmap.MaxUnknownRatio = 2
	runtimeConfig.NavigationAdapter.MaxXYSpeedMPS = 0
	runtimeConfig.NavigationAdapter.SetpointIntentTopic = runtimeConfig.FCUController.CmdVelTopic
	runtimeConfig.NavigationMission.MinCoverageGrowth = 2

	blockers := validateNavigationConfig(runtimeConfig)
	for _, expected := range []string{
		"nav2_tf_invalid:frame_missing",
		"navigation_scan_source_not_stabilized",
		"nav2_direct_fcu_cmd_vel_publisher",
		"navigation_adapter_direct_fcu_cmd_vel_publisher",
		"nav2_obstacle_layer_missing",
		"nav2_inflation_layer_missing",
		"navigation_costmap_topic_missing",
		"nav2_costmap_stale_threshold_invalid",
		"navigation_costmap_unknown_ratio_invalid",
		"navigation_uses_gazebo_truth_as_input",
		"navigation_adapter_limit_invalid",
		"navigation_coverage_threshold_invalid",
	} {
		if !containsString(blockers, expected) {
			t.Fatalf("expected blocker %q in %#v", expected, blockers)
		}
	}
}

func TestNavigationConfigRejectsCompetingCmdVelPublishers(t *testing.T) {
	runtimeConfig := validNavigationRuntimeConfig()
	runtimeConfig.Nav2.CmdVelTopic = "/ap/v1/cmd_vel"

	blockers := validateNavigationConfig(runtimeConfig)
	if !containsString(blockers, "nav2_direct_fcu_cmd_vel_publisher") {
		t.Fatalf("nav2 blockers = %#v", blockers)
	}

	runtimeConfig = validNavigationRuntimeConfig()
	runtimeConfig.NavigationAdapter.SetpointIntentTopic = "/ap/v1/cmd_vel"
	blockers = validateNavigationConfig(runtimeConfig)
	if !containsString(blockers, "navigation_adapter_direct_fcu_cmd_vel_publisher") {
		t.Fatalf("adapter blockers = %#v", blockers)
	}
}

func TestEvaluateResultGatesAddsNavigationCompletionPolicyBlockers(t *testing.T) {
	runtimeConfig := validNavigationRuntimeConfig()
	runtimeConfig.NavigationMission.CompletionPolicy = "hover_forever"

	evaluation := EvaluateResultGates(
		config.ProjectConfig{},
		runtimeConfig,
		Plan{TaskID: "navigation"},
		t.TempDir(),
		RuntimeSpecBundle{},
		RuntimeExecutionResult{},
		nil,
	)
	if !containsString(evaluation.Blockers, "navigation_completion_policy_invalid") {
		t.Fatalf("blockers = %#v", evaluation.Blockers)
	}
}

func TestEvaluateResultGatesAddsNavigationProbeBlockers(t *testing.T) {
	artifactDir := t.TempDir()
	if err := os.WriteFile(filepath.Join(artifactDir, "nav2_lifecycle_probe.json"), []byte(`{"ok":false,"status":"sampled","nav2_claim":"evaluated","nav2_lifecycle_active":false,"nav2_action_server_ready":false,"blockers":["nav2_lifecycle_inactive","nav2_action_unavailable"],"samples":{}}`), 0o644); err != nil {
		t.Fatal(err)
	}
	evaluation := EvaluateResultGates(
		config.ProjectConfig{},
		validNavigationRuntimeConfig(),
		Plan{
			TaskID: "navigation",
			Execution: helpers.ExecutionPlan{
				ROSProbes: []helpers.ROSProbePlan{{Name: "nav2_lifecycle_probe", OutputPath: "nav2_lifecycle_probe.json"}},
			},
		},
		artifactDir,
		RuntimeSpecBundle{},
		RuntimeExecutionResult{},
		nil,
	)
	for _, expected := range []string{"probe_output_not_ok:nav2_lifecycle_probe", "nav2_lifecycle_inactive", "nav2_action_unavailable"} {
		if !containsString(evaluation.Blockers, expected) {
			t.Fatalf("expected blocker %q in %#v", expected, evaluation.Blockers)
		}
	}
	if evaluation.Metrics.Nav2["nav2_lifecycle_active"] != false || evaluation.Metrics.Nav2["nav2_action_server_ready"] != false {
		t.Fatalf("nav2 metrics = %#v", evaluation.Metrics.Nav2)
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

func TestMetricSummaryIncludesP13NavigationMetrics(t *testing.T) {
	metrics := metricSummaryFromEvidence([]ProbeOutputSummary{
		{
			Name: "nav2_lifecycle_probe",
			Payload: map[string]any{
				"nav2_claim":               "evaluated",
				"nav2_lifecycle_active":    true,
				"nav2_action_server_ready": true,
				"tf":                       map[string]any{"map_to_odom": map[string]any{"valid": true}},
			},
		},
		{
			Name: "navigation_probe",
			Payload: map[string]any{
				"samples": map[string]any{
					"/navlab/navigation/status": map[string]any{
						"data": `{"claim":"evaluated","navigation_claim":"evaluated","frontier_claim":"evaluated","strategy":"bounded_frontier","frontier_candidates":[{"id":"p13_probe_1","source":"bounded_goal_slam_map_costmap"}],"accepted_frontiers":[{"id":"p13_probe_1"}],"rejected_frontiers":[{"id":"unreachable_probe","reason":"unreachable_blacklisted"}],"blacklisted_goals":["unreachable_probe"],"accepted_goals":3,"min_accepted_goals":3,"path_length_m":4.2,"min_path_length_m":4.0,"recovery_count":1,"return_home_policy":"return_home_then_land","goal_success_ratio":0.75,"coverage_growth":0.5,"min_coverage_growth":0.5,"uses_gazebo_truth_as_input":false}`,
					},
					"/navlab/navigation/adapter/status": map[string]any{
						"data": `{"claim":"evaluated","adapter_claim":"evaluated","active":true,"max_xy_speed_mps":0.25,"fixed_altitude_m":0.8,"stop_on_stale_costmap":true,"stop_on_stale_slam":true,"clamp_count":2,"hold_count":1,"intent_count":5,"hold_reason":""}`,
					},
					"/navlab/navigation/costmap_health": map[string]any{
						"data": `{"claim":"evaluated","costmap_claim":"evaluated","global_costmap_age_sec":0.2,"local_costmap_age_sec":0.1,"local_costmap_update_frequency_hz":5.0,"unknown_ratio":0.12,"obstacle_cells":8,"required_layers":["static_layer","obstacle_layer","inflation_layer"]}`,
					},
				},
			},
		},
	}, nil)
	if metrics.Navigation["accepted_goals"] == nil || metrics.Navigation["return_home_policy"] != "return_home_then_land" {
		t.Fatalf("navigation metrics = %#v", metrics.Navigation)
	}
	if metrics.Navigation["navigation_claim"] != "evaluated" {
		t.Fatalf("navigation claim metrics = %#v", metrics.Navigation)
	}
	if metrics.Navigation["frontier_claim"] != "evaluated" ||
		metrics.Navigation["frontier_candidates"] == nil ||
		metrics.Navigation["accepted_frontiers"] == nil ||
		metrics.Navigation["rejected_frontiers"] == nil ||
		metrics.Navigation["blacklisted_goals"] == nil ||
		metrics.Navigation["coverage_growth"] == nil ||
		metrics.Navigation["min_coverage_growth"] == nil {
		t.Fatalf("navigation frontier metrics = %#v", metrics.Navigation)
	}
	if metrics.Nav2["nav2_lifecycle_active"] != true || metrics.Nav2["nav2_action_server_ready"] != true {
		t.Fatalf("nav2 metrics = %#v", metrics.Nav2)
	}
	if metrics.NavigationAdapter["max_xy_speed_mps"] == nil || metrics.NavigationAdapter["clamp_count"] == nil {
		t.Fatalf("adapter metrics = %#v", metrics.NavigationAdapter)
	}
	if metrics.CostmapHealth["unknown_ratio"] == nil || metrics.CostmapHealth["required_layers"] == nil || metrics.CostmapHealth["local_costmap_update_frequency_hz"] == nil {
		t.Fatalf("costmap metrics = %#v", metrics.CostmapHealth)
	}
	if metrics.NavigationAdapter["adapter_claim"] != "evaluated" || metrics.CostmapHealth["costmap_claim"] != "evaluated" {
		t.Fatalf("claim metrics adapter=%#v costmap=%#v", metrics.NavigationAdapter, metrics.CostmapHealth)
	}
}

func TestMetricSummaryIncludesSLAMStabilityMetrics(t *testing.T) {
	artifactDir := t.TempDir()
	log := strings.Join([]string{
		"W0615 Dropped earlier points from trajectory",
		"W0615 rejected odom TF stamp=1.000000000 x=99.000 y=0.000 z=0.000 rejected=1",
		"terminate called after throwing an instance of 'std::length_error'",
	}, "\n")
	if err := os.WriteFile(filepath.Join(artifactDir, "slam_backend.runtime.log"), []byte(log), 0o644); err != nil {
		t.Fatal(err)
	}
	metrics := metricSummaryFromEvidence([]ProbeOutputSummary{
		{
			Name: "frame_contract_probe",
			Payload: map[string]any{
				"samples": map[string]any{
					"/navlab/slam/status": map[string]any{
						"data": `{"state":"publishing_cached_slam_tf_odom","ready":true,"mode":"slam_tf","tf":{"count":12,"received_count":15,"rejected_count":3,"rejection_ratio":0.2,"max_observed_jump_m":2.5,"max_accepted_jump_m":0.4,"max_rejected_jump_m":2.5,"max_allowed_jump_m":2.0},"output":{"odom_topic":"/slam/odom","odom_count":12}}`,
					},
				},
			},
		},
	}, nil, artifactDir)
	tf, ok := metrics.SLAM["tf"].(map[string]any)
	if !ok || tf["rejected_count"] != float64(3) || tf["max_rejected_jump_m"] != 2.5 {
		t.Fatalf("slam tf metrics = %#v", metrics.SLAM)
	}
	if metrics.SLAMRuntimeLog["dropped_earlier_points_count"] != 1 ||
		metrics.SLAMRuntimeLog["rejected_odom_tf_log_count"] != 1 ||
		metrics.SLAMRuntimeLog["std_length_error_count"] != 1 ||
		metrics.SLAMRuntimeLog["warning_count"] != 2 ||
		metrics.SLAMRuntimeLog["ok"] != false {
		t.Fatalf("slam runtime log metrics = %#v", metrics.SLAMRuntimeLog)
	}
	if metrics.MetricEvidenceSources["slam_runtime_log"] != "slam_backend.runtime.log" {
		t.Fatalf("metric evidence sources = %#v", metrics.MetricEvidenceSources)
	}
}

func TestSLAMRuntimeLogBlockersFailClosed(t *testing.T) {
	blockers := slamRuntimeLogBlockers(map[string]any{
		"ok":                     false,
		"std_length_error_count": 1,
		"fatal_count":            0,
		"error_count":            1,
	})

	for _, want := range []string{
		"slam_runtime_unhealthy",
		"slam_runtime_std_length_error",
		"slam_runtime_error",
	} {
		if !stringSliceContains(blockers, want) {
			t.Fatalf("blockers = %#v, want %s", blockers, want)
		}
	}
}

func TestMetricSummaryIncludesFCUSetpointEvidence(t *testing.T) {
	metrics := metricSummaryFromEvidence([]ProbeOutputSummary{
		{
			Name: "fcu_probe",
			Payload: map[string]any{
				"samples": map[string]any{
					"/navlab/fcu/controller/status": map[string]any{
						"data": `{"ready":true,"state":"hover_hold","pose_samples":12,"control_route":"mavlink_bootstrap_plus_dds_cmd_vel","takeoff_alt_m":0.5,"bootstrap_ready":true,"cmd_vel_publish_count":42,"mavlink_setpoint_count":21,"mavlink_setpoint_error":"","mavlink_local_position_count":9,"fcu_mode_window":{"ok":true},"bootstrap":{"takeoff":{"ok":true,"ack_accepted_seen":true,"height":{"height_m":0.32,"target_min_m":0.2}}}}`,
					},
					"/navlab/fcu/setpoint/output": map[string]any{
						"data": `{"ready":true,"state":"hold_position","intent_topic":"/navlab/fcu/setpoint/intent","cmd_vel_topic":"/ap/v1/cmd_vel","setpoint_intent_samples":5,"cmd_vel_publish_count":42,"mavlink_setpoint_count":21,"mavlink_setpoint_error":"","mavlink_local_position_count":9,"path_length_m":1.2,"min_path_length_m":0.35}`,
					},
				},
			},
		},
	}, nil)
	if metrics.Controller["mavlink_setpoint_count"] != float64(21) ||
		metrics.Controller["mavlink_local_position_count"] != float64(9) ||
		metrics.Controller["bootstrap_ready"] != true ||
		metrics.Setpoint["mavlink_setpoint_count"] != float64(21) ||
		metrics.Setpoint["mavlink_local_position_count"] != float64(9) ||
		metrics.Setpoint["setpoint_intent_samples"] != float64(5) ||
		metrics.Controller["bootstrap"] == nil {
		t.Fatalf("fcu setpoint metrics controller=%#v setpoint=%#v", metrics.Controller, metrics.Setpoint)
	}
}

func TestHoverTakeoffBlockersRequireObservedHeight(t *testing.T) {
	passing := map[string]any{
		"takeoff_alt_m": 0.5,
		"bootstrap": map[string]any{
			"takeoff": map[string]any{
				"ok":                true,
				"ack_accepted_seen": true,
				"height": map[string]any{
					"height_m":     0.42,
					"target_min_m": 0.175,
				},
			},
		},
	}
	if blockers := hoverTakeoffBlockers("hover", passing); len(blockers) != 0 {
		t.Fatalf("blockers = %#v, want none", blockers)
	}

	failing := map[string]any{
		"takeoff_alt_m": 0.5,
		"bootstrap": map[string]any{
			"takeoff": map[string]any{
				"ok":                true,
				"ack_accepted_seen": true,
				"height": map[string]any{
					"height_m":     0.0,
					"target_min_m": 0.175,
				},
			},
		},
	}
	if blockers := hoverTakeoffBlockers("hover", failing); !stringSliceContains(blockers, "hover_takeoff_height_not_observed") {
		t.Fatalf("blockers = %#v, want hover_takeoff_height_not_observed", blockers)
	}

	tooLow := map[string]any{
		"takeoff_alt_m": 0.5,
		"bootstrap": map[string]any{
			"takeoff": map[string]any{
				"ok":                true,
				"ack_accepted_seen": true,
				"height": map[string]any{
					"height_m":     0.19,
					"target_min_m": 0.175,
				},
			},
		},
	}
	if blockers := hoverTakeoffBlockers("hover", tooLow); !stringSliceContains(blockers, "hover_takeoff_target_altitude_not_reached") {
		t.Fatalf("blockers = %#v, want hover_takeoff_target_altitude_not_reached", blockers)
	}
}

func validNavigationRuntimeConfig() config.TaskRuntimeConfig {
	return config.TaskRuntimeConfig{
		Nav2: config.Nav2Config{
			Enabled:          true,
			Profile:          "indoor_2d",
			GlobalFrame:      "map",
			OdomFrame:        "odom",
			BaseFrame:        "base_link",
			ScanTopic:        "/scan",
			MapTopic:         "/map",
			CmdVelTopic:      "/navlab/navigation/cmd_vel",
			BTXML:            "navigate_to_pose_w_replanning_and_recovery.xml",
			PlannerPlugin:    "GridBased",
			ControllerPlugin: "FollowPath",
			UseSimTime:       true,
			Costmap: config.Nav2CostmapConfig{
				GlobalCostmapTopic: "/global_costmap/costmap",
				LocalCostmapTopic:  "/local_costmap/costmap",
				RequiredLayers:     []string{"static_layer", "obstacle_layer", "inflation_layer"},
				MaxCostmapAgeSec:   1.5,
				MinObstacleCells:   1,
				MaxUnknownRatio:    0.35,
				HealthTopic:        "/navlab/navigation/costmap_health",
			},
		},
		NavigationAdapter: config.NavigationAdapterConfig{
			SetpointIntentTopic: "/navlab/fcu/setpoint/intent",
			StatusTopic:         "/navlab/navigation/adapter/status",
			MaxXYSpeedMPS:       0.25,
			MaxYawRateDPS:       35,
			MaxAccelMPS2:        0.35,
			FixedAltitudeM:      0.8,
			StopOnStaleCostmap:  true,
			StopOnStaleSlam:     true,
		},
		NavigationMission: config.NavigationMissionConfig{
			Strategy:            "bounded_frontier",
			GoalFrame:           "map",
			StatusTopic:         "/navlab/navigation/status",
			NavigationWindowSec: 120,
			MinPathLengthM:      4,
			MinCoverageGrowth:   0.5,
			MinAcceptedGoals:    3,
			CompletionPolicy:    helpers.PolicyLandInPlace,
			ReturnHomePolicy:    helpers.PolicyLandInPlace,
			NavigationClaim:     "evaluated",
			ExitGoal:            config.NavigationGoalConfig{ID: "maze_exit", XM: 1.5, YM: -0.5, YawRad: 0},
			BoundedGoals:        []config.NavigationGoalConfig{{ID: "p13_probe_1", XM: 1, YM: 0, YawRad: 0}},
			HomeGoal:            config.NavigationGoalConfig{ID: "home"},
		},
		FCUController: config.FCUControllerConfig{
			CmdVelTopic:           "/ap/v1/cmd_vel",
			ControllerStatusTopic: "/navlab/fcu/controller/status",
			OwnerStatusTopic:      "/navlab/fcu/owner/status",
		},
		ScanStabilization: config.ScanStabilizationConfig{
			OutputScanTopic: "/scan",
		},
		Landing: config.LandingConfig{
			ExplorationPolicy: helpers.PolicyReturnHomeThenLand,
			NavigationPolicy:  helpers.PolicyLandInPlace,
			DefaultPolicy:     helpers.PolicyLandInPlace,
		},
	}
}
