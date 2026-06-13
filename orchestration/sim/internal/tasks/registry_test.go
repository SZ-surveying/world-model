package tasks

import (
	"testing"

	"navlab/orchestration-sim/internal/config"
	"navlab/orchestration-sim/internal/tasks/helpers"
)

func TestDefaultRegistryConfiguresKnownTask(t *testing.T) {
	registry := DefaultRegistry()
	task, err := registry.ConfigureOne(config.TaskConfig{
		ID:          "hover",
		Family:      "sim",
		Description: "custom hover",
		Task: config.TaskParameters{
			DurationSec:       90,
			SimulationProfile: "ideal",
		},
	})
	if err != nil {
		t.Fatalf("ConfigureOne() error = %v", err)
	}
	plan, err := task.Plan(PlanOptions{DurationSec: 10, SimulationProfile: "realistic"}, helpers.DefaultRegistry())
	if err != nil {
		t.Fatalf("Plan() error = %v", err)
	}
	if plan.DurationSec != 10 {
		t.Fatalf("DurationSec = %v, want 10", plan.DurationSec)
	}
	if plan.SimulationProfile != "realistic" {
		t.Fatalf("SimulationProfile = %q, want realistic", plan.SimulationProfile)
	}
	if len(plan.Steps) == 0 {
		t.Fatal("plan steps are empty")
	}
	if len(plan.Helpers) == 0 {
		t.Fatal("plan helpers are empty")
	}
}

func TestDefaultRegistryRejectsUnregisteredTask(t *testing.T) {
	registry := DefaultRegistry()
	_, err := registry.ConfigureOne(config.TaskConfig{ID: "preflight", Family: "sim"})
	if err == nil {
		t.Fatal("ConfigureOne(preflight) error = nil, want error")
	}
}
