package main

import (
	"encoding/json"
	"errors"
	"flag"
	"fmt"
	"os"
	"path/filepath"

	"navlab/orchestration-sim/internal/config"
	"navlab/orchestration-sim/internal/tasks"
)

func main() {
	configPath := flag.String("config", "config.toml", "sim orchestration TOML config path")
	outputPath := flag.String("output", "", "write replay JSON to this path instead of stdout")
	flag.Parse()
	if flag.NArg() != 1 {
		fmt.Fprintln(os.Stderr, "usage: hover-gate-replay [--config config.toml] [--output gate_replay.json] <artifact-dir>")
		os.Exit(2)
	}
	if err := run(*configPath, flag.Arg(0), *outputPath); err != nil {
		fmt.Fprintln(os.Stderr, "error:", err)
		os.Exit(1)
	}
}

type taskPlanFile struct {
	Plan tasks.Plan `json:"plan"`
}

type replayOutput struct {
	SchemaVersion  string               `json:"schemaVersion"`
	ArtifactDir    string               `json:"artifact_dir"`
	TaskID         string               `json:"task_id"`
	RunID          string               `json:"run_id"`
	GateEvaluation tasks.GateEvaluation `json:"gate_evaluation"`
}

func run(configPath string, artifactDir string, outputPath string) error {
	artifactDir = filepath.Clean(artifactDir)
	summary, err := readJSON[tasks.LiveRunSummary](filepath.Join(artifactDir, "summary.json"))
	if err != nil {
		return err
	}
	taskPlan, err := readJSON[taskPlanFile](filepath.Join(artifactDir, "task_plan.json"))
	if err != nil {
		return err
	}
	runtimeSpecs, err := readJSON[tasks.RuntimeSpecBundle](filepath.Join(artifactDir, "runtime_plan.json"))
	if err != nil {
		return err
	}

	loader := config.NewLoader(configPath)
	project, err := loader.LoadProject()
	if err != nil {
		return fmt.Errorf("load project config: %w", err)
	}
	taskConfig, err := loader.LoadTask(project, taskPlan.Plan.TaskID)
	if err != nil {
		return fmt.Errorf("load task config %q: %w", taskPlan.Plan.TaskID, err)
	}
	runtimeConfig, err := config.BuildTaskRuntimeConfig(project, taskConfig)
	if err != nil {
		return fmt.Errorf("build runtime config: %w", err)
	}
	runtimeConfig, err = tasks.ApplySimulationProfile(runtimeConfig, taskPlan.Plan)
	if err != nil {
		return fmt.Errorf("apply simulation profile: %w", err)
	}

	var executionErr error
	if summary.RuntimeError != "" {
		executionErr = errors.New(summary.RuntimeError)
	}
	gate := tasks.EvaluateResultGates(
		project,
		runtimeConfig,
		taskPlan.Plan,
		artifactDir,
		runtimeSpecs,
		summary.RuntimeExecution,
		executionErr,
	)
	out := replayOutput{
		SchemaVersion:  "navlab.orchestration.gate_replay.v1",
		ArtifactDir:    artifactDir,
		TaskID:         taskPlan.Plan.TaskID,
		RunID:          summary.RunID,
		GateEvaluation: gate,
	}
	data, err := json.MarshalIndent(out, "", "  ")
	if err != nil {
		return err
	}
	data = append(data, '\n')
	if outputPath != "" {
		return os.WriteFile(outputPath, data, 0o644)
	}
	_, err = os.Stdout.Write(data)
	return err
}

func readJSON[T any](path string) (T, error) {
	var value T
	data, err := os.ReadFile(path)
	if err != nil {
		return value, fmt.Errorf("read %s: %w", path, err)
	}
	if err := json.Unmarshal(data, &value); err != nil {
		return value, fmt.Errorf("parse %s: %w", path, err)
	}
	return value, nil
}
