package config

import (
	"path/filepath"
	"testing"
)

func TestBuildTaskRuntimeConfigAppliesTaskOverrides(t *testing.T) {
	loader := NewLoader(filepath.Join("..", "..", "config.toml"))
	project, err := loader.LoadProject()
	if err != nil {
		t.Fatal(err)
	}
	task, err := loader.LoadTask(project, "hover")
	if err != nil {
		t.Fatal(err)
	}

	runtimeConfig, err := BuildTaskRuntimeConfig(project, task)
	if err != nil {
		t.Fatal(err)
	}

	if runtimeConfig.FCUController.TakeoffAltM != 0.5 {
		t.Fatalf("takeoff alt = %v", runtimeConfig.FCUController.TakeoffAltM)
	}
	if runtimeConfig.SlamHover.HoverWindowSec != 18 {
		t.Fatalf("hover window = %v", runtimeConfig.SlamHover.HoverWindowSec)
	}
	if runtimeConfig.Landing.HoverPolicy != "land_in_place" {
		t.Fatalf("hover landing policy = %q", runtimeConfig.Landing.HoverPolicy)
	}
	if runtimeConfig.FrameContract.MapFrameID != "map" {
		t.Fatalf("default frame contract not preserved: %#v", runtimeConfig.FrameContract)
	}
}

func TestBuildTaskRuntimeConfigAppliesExplorationOverrides(t *testing.T) {
	loader := NewLoader(filepath.Join("..", "..", "config.toml"))
	project, err := loader.LoadProject()
	if err != nil {
		t.Fatal(err)
	}
	task, err := loader.LoadTask(project, "exploration")
	if err != nil {
		t.Fatal(err)
	}
	runtimeConfig, err := BuildTaskRuntimeConfig(project, task)
	if err != nil {
		t.Fatal(err)
	}
	if runtimeConfig.ExplorationGate.MinAcceptedGoals != 3 {
		t.Fatalf("min accepted goals = %d", runtimeConfig.ExplorationGate.MinAcceptedGoals)
	}
	if runtimeConfig.Landing.ExplorationPolicy != "return_home_then_land" {
		t.Fatalf("exploration landing policy = %q", runtimeConfig.Landing.ExplorationPolicy)
	}
}

func TestBuildTaskRuntimeConfigAppliesScanRobustnessOverrides(t *testing.T) {
	loader := NewLoader(filepath.Join("..", "..", "config.toml"))
	project, err := loader.LoadProject()
	if err != nil {
		t.Fatal(err)
	}
	task, err := loader.LoadTask(project, "scan-robustness")
	if err != nil {
		t.Fatal(err)
	}
	runtimeConfig, err := BuildTaskRuntimeConfig(project, task)
	if err != nil {
		t.Fatal(err)
	}
	if !runtimeConfig.ScanRobustness.Live {
		t.Fatalf("scan robustness live = false")
	}
	if runtimeConfig.AirframeDisturbance.Profile != "ideal" {
		t.Fatalf("airframe profile = %q", runtimeConfig.AirframeDisturbance.Profile)
	}
	if runtimeConfig.Landing.ScanRobustnessPolicy != "land_in_place" {
		t.Fatalf("scan landing policy = %q", runtimeConfig.Landing.ScanRobustnessPolicy)
	}
}
