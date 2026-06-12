package tasks

import (
	"os"
	"path/filepath"
	"testing"

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
