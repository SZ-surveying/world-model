package tasks

import (
	"fmt"
	"strings"
	"time"

	"navlab/orchestration-sim/internal/config"
)

type LiveRunSummary struct {
	SchemaVersion             string                     `json:"schemaVersion"`
	OK                        bool                       `json:"ok"`
	Blocked                   bool                       `json:"blocked"`
	Status                    string                     `json:"status"`
	ExitCode                  int                        `json:"exitCode"`
	SummaryPath               string                     `json:"summaryPath,omitempty"`
	Blockers                  []TaskResultBlocker        `json:"blockers"`
	BlockerCodes              []string                   `json:"blockerCodes"`
	Warnings                  []string                   `json:"warnings"`
	AcceptanceStage           string                     `json:"acceptance_stage"`
	TaskID                    string                     `json:"task_id"`
	RunID                     string                     `json:"run_id"`
	ArtifactDir               string                     `json:"artifact_dir"`
	SourceEvidence            map[string]any             `json:"sourceEvidence"`
	Metrics                   map[string]any             `json:"metrics"`
	Evidence                  map[string]any             `json:"evidence"`
	Details                   map[string]any             `json:"details"`
	StartedAt                 string                     `json:"startedAt"`
	FinishedAt                string                     `json:"finishedAt"`
	DurationSec               float64                    `json:"duration_sec"`
	SimulationProfile         string                     `json:"simulation_profile"`
	Stage1ProfileResult       Stage1ProfileResult        `json:"stage1_profile_result"`
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

type TaskResultBlocker struct {
	Code    string `json:"code"`
	Message string `json:"message"`
	Source  string `json:"source"`
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
	ok := len(blockers) == 0
	now := time.Now().UTC().Format(time.RFC3339)
	gateParity := GateParitySummary{
		Status: "evaluated_from_runtime_artifacts",
		Notes: []string{
			"Go sim live runner executes runtime service/probe/rosbag specs.",
			"Go sim evaluates runtime errors, probe outputs, rosbag metadata, task config checks, and landing acceptance from artifacts.",
		},
	}
	summary := LiveRunSummary{
		SchemaVersion:             "navlab.orchestration.task_result.v1",
		OK:                        ok,
		Blocked:                   !ok,
		Status:                    taskStatus(ok, executionErr),
		ExitCode:                  taskExitCode(ok, executionErr),
		Blockers:                  taskResultBlockers(blockers),
		BlockerCodes:              append([]string(nil), blockers...),
		Warnings:                  []string{},
		AcceptanceStage:           "simulation",
		TaskID:                    plan.TaskID,
		RunID:                     runID,
		ArtifactDir:               artifactDir,
		SourceEvidence:            simSourceEvidence(project),
		Metrics:                   liveMetrics(gateEvaluation),
		Evidence:                  liveEvidence(gateEvaluation, execution),
		Details:                   map[string]any{},
		StartedAt:                 now,
		FinishedAt:                now,
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
		CreatedAt:        now,
	}
	summary.Stage1ProfileResult = Stage1ProfileResultFromSummary(summary)
	return summary
}

func taskStatus(ok bool, executionErr error) string {
	if ok {
		return "TASK_STATUS_OK"
	}
	if executionErr != nil {
		return "TASK_STATUS_ERROR"
	}
	return "TASK_STATUS_BLOCKED"
}

func taskExitCode(ok bool, executionErr error) int {
	if ok {
		return 0
	}
	if executionErr != nil {
		return 1
	}
	return 20
}

func taskResultBlockers(blockers []string) []TaskResultBlocker {
	result := make([]TaskResultBlocker, 0, len(blockers))
	for _, blocker := range blockers {
		code := blockerCode(blocker)
		result = append(result, TaskResultBlocker{
			Code:    code,
			Message: blocker,
			Source:  blockerSource(code),
		})
	}
	return result
}

func blockerCode(blocker string) string {
	code := blocker
	if index := strings.Index(code, ":"); index >= 0 {
		code = code[:index]
	}
	code = strings.TrimSpace(code)
	if code == "" {
		return "unknown_blocker"
	}
	return code
}

func blockerSource(code string) string {
	switch {
	case strings.HasPrefix(code, "runtime_"):
		return "runtime"
	case strings.HasPrefix(code, "probe_"):
		return "probe"
	case strings.HasPrefix(code, "rosbag_"):
		return "rosbag"
	case strings.Contains(code, "landing"):
		return "landing"
	default:
		return "gate"
	}
}

func simSourceEvidence(project config.ProjectConfig) map[string]any {
	return map[string]any{
		"runtimeDomain":                 "RUNTIME_DOMAIN_SIM",
		"scanSource":                    simContractScanSource(project.Sensor.ScanSource),
		"imuSource":                     fallbackString(project.RangefinderIMU.IMUSourceRoute, "official_gazebo_imu_bridge"),
		"rangefinderSource":             "ardupilot_serial7_benewake_tfmini",
		"rangefinderSimulationFidelity": "benewake_serial_emulated",
		"slamSource":                    fallbackString(project.Slam.Backend, "cartographer"),
		"usesTruthAsControlInput":       false,
	}
}

func fallbackString(value string, fallback string) string {
	if value == "" {
		return fallback
	}
	return value
}

func simContractScanSource(value string) string {
	switch strings.TrimSpace(value) {
	case "", "x2_virtual_serial":
		return "gazebo_x2_virtual_serial"
	default:
		return value
	}
}

func liveMetrics(gateEvaluation GateEvaluation) map[string]any {
	return map[string]any{
		"gate": gateEvaluation.Metrics,
	}
}

func liveEvidence(gateEvaluation GateEvaluation, execution RuntimeExecutionResult) map[string]any {
	return map[string]any{
		"probeOutputs":    gateEvaluation.ProbeOutputs,
		"rosbagProfiles":  gateEvaluation.RosbagProfiles,
		"runtimeHandles":  execution.ServiceHandles,
		"rosbagHandles":   execution.RosbagHandles,
		"runtimeProbes":   execution.ProbeResults,
		"landingEvidence": gateEvaluation.Landing,
	}
}
