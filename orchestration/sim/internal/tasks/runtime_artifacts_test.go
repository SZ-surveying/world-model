package tasks

import (
	"os"
	"path/filepath"
	"reflect"
	"testing"

	"github.com/pelletier/go-toml/v2"

	"navlab/orchestration-sim/internal/config"
	"navlab/orchestration-sim/internal/tasks/helpers"
)

func TestGenerateRuntimeArtifactsFromConfiguredTasks(t *testing.T) {
	tests := []struct {
		name      string
		taskID    string
		wantFiles []string
	}{
		{
			name:   "hover",
			taskID: "hover",
			wantFiles: []string{
				"bridge_override.yaml",
				"vendor_profile.yaml",
				"gazebo_sensor_runtime.toml",
				"rangefinder_probe.py",
				"imu_probe.py",
				"slam_runtime.toml",
				"fcu_controller_runtime.toml",
				"fcu_controller_runtime.py",
				"frame_contract_runtime.toml",
				"frame_contract_probe.py",
				"slam_hover_runtime.toml",
				"slam_hover_probe.py",
			},
		},
		{
			name:   "exploration",
			taskID: "exploration",
			wantFiles: []string{
				"exploration_runtime.toml",
				"exploration_probe.py",
				"motion_foxglove_notes.md",
			},
		},
		{
			name:   "scan-robustness",
			taskID: "scan-robustness",
			wantFiles: []string{
				"scan_stabilization_runtime.toml",
				"stabilization_status_probe.py",
				"scan_robustness_runtime.toml",
				"scan_robustness_bridge_override.yaml",
				"airframe_disturbance_probe.py",
			},
		},
	}

	loader := config.NewLoader("../../config.toml")
	project, err := loader.LoadProject()
	if err != nil {
		t.Fatalf("LoadProject() error = %v", err)
	}
	registry := DefaultRegistry()
	helperRegistry := helpers.DefaultRegistry()

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			taskConfig, err := loader.LoadTask(project, tt.taskID)
			if err != nil {
				t.Fatalf("LoadTask(%q) error = %v", tt.taskID, err)
			}
			runtimeConfig, err := config.BuildTaskRuntimeConfig(project, taskConfig)
			if err != nil {
				t.Fatalf("BuildTaskRuntimeConfig(%q) error = %v", tt.taskID, err)
			}
			task, err := registry.ConfigureOne(taskConfig)
			if err != nil {
				t.Fatalf("ConfigureOne(%q) error = %v", tt.taskID, err)
			}
			plan, err := task.Plan(PlanOptions{}, helperRegistry)
			if err != nil {
				t.Fatalf("Plan(%q) error = %v", tt.taskID, err)
			}
			runtimeConfig, err = ApplySimulationProfile(runtimeConfig, plan)
			if err != nil {
				t.Fatalf("ApplySimulationProfile(%q) error = %v", tt.taskID, err)
			}

			artifactDir := t.TempDir()
			generated, err := GenerateRuntimeArtifacts(project, plan, runtimeConfig, artifactDir)
			if err != nil {
				t.Fatalf("GenerateRuntimeArtifacts(%q) error = %v", tt.taskID, err)
			}
			if len(generated) == 0 {
				t.Fatalf("GenerateRuntimeArtifacts(%q) generated no artifacts", tt.taskID)
			}
			for _, name := range tt.wantFiles {
				assertFileExists(t, filepath.Join(artifactDir, name))
			}
		})
	}
}

func TestGenerateRuntimeArtifactsUsesSimulationProfileForScanRobustness(t *testing.T) {
	loader := config.NewLoader("../../config.toml")
	project, err := loader.LoadProject()
	if err != nil {
		t.Fatalf("LoadProject() error = %v", err)
	}
	taskConfig, err := loader.LoadTask(project, "scan-robustness")
	if err != nil {
		t.Fatalf("LoadTask(scan-robustness) error = %v", err)
	}
	runtimeConfig, err := config.BuildTaskRuntimeConfig(project, taskConfig)
	if err != nil {
		t.Fatalf("BuildTaskRuntimeConfig(scan-robustness) error = %v", err)
	}
	task, err := DefaultRegistry().ConfigureOne(taskConfig)
	if err != nil {
		t.Fatalf("ConfigureOne(scan-robustness) error = %v", err)
	}
	plan, err := task.Plan(PlanOptions{SimulationProfile: "realistic"}, helpers.DefaultRegistry())
	if err != nil {
		t.Fatalf("Plan(scan-robustness) error = %v", err)
	}
	runtimeConfig, err = ApplySimulationProfile(runtimeConfig, plan)
	if err != nil {
		t.Fatalf("ApplySimulationProfile(scan-robustness) error = %v", err)
	}

	artifactDir := t.TempDir()
	if _, err := GenerateRuntimeArtifacts(project, plan, runtimeConfig, artifactDir); err != nil {
		t.Fatalf("GenerateRuntimeArtifacts(scan-robustness) error = %v", err)
	}
	data, err := os.ReadFile(filepath.Join(artifactDir, "scan_robustness_runtime.toml"))
	if err != nil {
		t.Fatalf("ReadFile(scan_robustness_runtime.toml) error = %v", err)
	}
	var generated struct {
		AirframeDisturbance struct {
			Runtime struct {
				Profile          string
				RequiredProfiles []string
			}
		} `toml:"airframe_disturbance"`
		AirframeDisturbanceGate struct {
			Runtime struct {
				RequiredProfiles []string `toml:"required_profiles"`
			}
		} `toml:"airframe_disturbance_gate"`
	}
	if err := toml.Unmarshal(data, &generated); err != nil {
		t.Fatalf("toml.Unmarshal(scan_robustness_runtime.toml) error = %v", err)
	}
	if generated.AirframeDisturbance.Runtime.Profile != "realistic" {
		t.Fatalf("runtime profile = %q", generated.AirframeDisturbance.Runtime.Profile)
	}
	wantProfiles := []string{"ideal", "realistic"}
	if !reflect.DeepEqual(generated.AirframeDisturbance.Runtime.RequiredProfiles, wantProfiles) {
		t.Fatalf("runtime required profiles = %#v, want %#v", generated.AirframeDisturbance.Runtime.RequiredProfiles, wantProfiles)
	}
	if !reflect.DeepEqual(generated.AirframeDisturbanceGate.Runtime.RequiredProfiles, wantProfiles) {
		t.Fatalf("gate required profiles = %#v, want %#v", generated.AirframeDisturbanceGate.Runtime.RequiredProfiles, wantProfiles)
	}
}

func assertFileExists(t *testing.T, path string) {
	t.Helper()
	info, err := os.Stat(path)
	if err != nil {
		t.Fatalf("expected file %s: %v", path, err)
	}
	if info.IsDir() {
		t.Fatalf("expected file %s, got directory", path)
	}
}
