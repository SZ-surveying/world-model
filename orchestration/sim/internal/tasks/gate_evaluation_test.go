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
