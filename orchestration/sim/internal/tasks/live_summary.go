package tasks

import (
	"fmt"
	"time"

	"navlab/orchestration-sim/internal/config"
)

type LiveRunSummary struct {
	OK                        bool                       `json:"ok"`
	Blocked                   bool                       `json:"blocked"`
	Blockers                  []string                   `json:"blockers"`
	AcceptanceStage           string                     `json:"acceptance_stage"`
	TaskID                    string                     `json:"task_id"`
	RunID                     string                     `json:"run_id"`
	ArtifactDir               string                     `json:"artifact_dir"`
	DurationSec               float64                    `json:"duration_sec"`
	SimulationProfile         string                     `json:"simulation_profile"`
	RuntimeMode               string                     `json:"runtime_mode"`
	Backend                   string                     `json:"backend"`
	GeneratedRuntimeArtifacts []GeneratedRuntimeArtifact `json:"generated_runtime_artifacts"`
	RuntimeSpecCounts         RuntimeSpecCounts          `json:"runtime_spec_counts"`
	RuntimeExecution          RuntimeExecutionResult     `json:"runtime_execution"`
	RuntimeError              string                     `json:"runtime_error,omitempty"`
	ResultGates               []ResultGateSummary        `json:"result_gates"`
	GateParity                GateParitySummary          `json:"gate_parity"`
	GateEvaluation            GateEvaluation             `json:"gate_evaluation"`
	CreatedAt                 string                     `json:"created_at"`
}

type RuntimeSpecCounts struct {
	Services int `json:"services"`
	Probes   int `json:"probes"`
	Rosbags  int `json:"rosbags"`
}

type ResultGateSummary struct {
	HelperID string   `json:"helper_id"`
	Name     string   `json:"name"`
	Inputs   []string `json:"inputs"`
	Outputs  []string `json:"outputs"`
	Status   string   `json:"status"`
}

type GateParitySummary struct {
	Status string   `json:"status"`
	Notes  []string `json:"notes"`
}

func BuildLiveRunSummary(
	project config.ProjectConfig,
	runtimeConfig config.TaskRuntimeConfig,
	plan Plan,
	runID string,
	artifactDir string,
	generatedArtifacts []GeneratedRuntimeArtifact,
	runtimeSpecs RuntimeSpecBundle,
	execution RuntimeExecutionResult,
	executionErr error,
) LiveRunSummary {
	blockers := []string{}
	runtimeError := ""
	if executionErr != nil {
		runtimeError = executionErr.Error()
		blockers = append(blockers, fmt.Sprintf("runtime_execution_failed:%s", executionErr.Error()))
	}
	resultGates := make([]ResultGateSummary, 0, len(plan.Execution.ResultGates))
	for _, gate := range plan.Execution.ResultGates {
		resultGates = append(resultGates, ResultGateSummary{
			HelperID: gate.HelperID,
			Name:     gate.Name,
			Inputs:   append([]string(nil), gate.Inputs...),
			Outputs:  append([]string(nil), gate.Outputs...),
			Status:   gate.Status,
		})
	}
	gateEvaluation := EvaluateResultGates(project, runtimeConfig, plan, artifactDir, runtimeSpecs, execution, executionErr)
	blockers = append(blockers, gateEvaluation.Blockers...)
	blockers = uniqueStrings(blockers)
	gateParity := GateParitySummary{
		Status: "evaluated_from_runtime_artifacts",
		Notes: []string{
			"Go sim live runner executes runtime service/probe/rosbag specs.",
			"Go sim evaluates runtime errors, probe outputs, rosbag metadata, task config checks, and landing acceptance from artifacts.",
		},
	}
	return LiveRunSummary{
		OK:                        len(blockers) == 0,
		Blocked:                   len(blockers) > 0,
		Blockers:                  blockers,
		AcceptanceStage:           "simulation",
		TaskID:                    plan.TaskID,
		RunID:                     runID,
		ArtifactDir:               artifactDir,
		DurationSec:               plan.DurationSec,
		SimulationProfile:         plan.SimulationProfile,
		RuntimeMode:               project.Runtime.Mode,
		Backend:                   project.Runtime.Backend,
		GeneratedRuntimeArtifacts: append([]GeneratedRuntimeArtifact(nil), generatedArtifacts...),
		RuntimeSpecCounts: RuntimeSpecCounts{
			Services: len(runtimeSpecs.Services),
			Probes:   len(runtimeSpecs.Probes),
			Rosbags:  len(runtimeSpecs.Rosbags),
		},
		RuntimeExecution: execution,
		RuntimeError:     runtimeError,
		ResultGates:      resultGates,
		GateParity:       gateParity,
		GateEvaluation:   gateEvaluation,
		CreatedAt:        time.Now().UTC().Format(time.RFC3339),
	}
}
