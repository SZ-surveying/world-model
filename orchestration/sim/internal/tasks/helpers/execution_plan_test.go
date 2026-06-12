package helpers

import (
	"os"
	"path/filepath"
	"strings"
	"testing"

	"navlab/orchestration-sim/internal/config"
)

func TestBuildExecutionPlanIncludesDeepRuntimeHelpers(t *testing.T) {
	task := config.TaskConfig{
		ID:          "scan-robustness",
		Family:      "sim",
		Description: "scan robustness",
		Task: config.TaskParameters{
			DurationSec:       240,
			SimulationProfile: "ideal",
		},
		Sections: map[string]any{
			"scan_robustness": map[string]any{"live": true},
		},
	}
	definitions, err := DefaultRegistry().Resolve([]string{
		"sensors",
		"slam",
		"fcu-controller",
		"frame-contract",
		"slam-hover",
		"scan-stabilization",
		"scan-robustness-workflow",
	})
	if err != nil {
		t.Fatal(err)
	}

	plan, err := BuildExecutionPlan(task, 240, "ideal", definitions)
	if err != nil {
		t.Fatal(err)
	}

	assertHasService(t, plan, "p4_controller")
	assertHasProbe(t, plan, "rangefinder_probe")
	assertHasProbe(t, plan, "frame_contract_probe")
	assertHasProbe(t, plan, "slam_hover_probe")
	assertHasProbe(t, plan, "stabilization_status_probe")
	assertHasRosbag(t, plan, "p11_scan_stabilization_rosbag")
	assertHasRosbag(t, plan, "p12_airframe_disturbance_rosbag")
	assertHasGate(t, plan, "airframe_disturbance_gate")
}

func TestRuntimeSpecsGenerateScriptsAndConfigs(t *testing.T) {
	dir := t.TempDir()
	if err := WriteP2SensorConfig(filepath.Join(dir, "sensor.toml"), "vendor.yaml", DefaultP2SensorSpec()); err != nil {
		t.Fatal(err)
	}
	if err := WriteP4RuntimeConfig(filepath.Join(dir, "fcu.toml"), DefaultFCUControllerSpec()); err != nil {
		t.Fatal(err)
	}
	if err := WriteP11RuntimeConfig(filepath.Join(dir, "p11.toml"), DefaultScanStabilizationSpec()); err != nil {
		t.Fatal(err)
	}
	script, err := FCUControllerRuntimeScript(DefaultFCUControllerSpec(), 90)
	if err != nil {
		t.Fatal(err)
	}
	if !strings.Contains(script, "navlab_fcu_controller") || !strings.Contains(script, "controller_status_topic") {
		t.Fatalf("controller script missing expected content:\n%s", script)
	}
	probe, err := FrameContractProbeScript(DefaultFrameContractSpec())
	if err != nil {
		t.Fatal(err)
	}
	if !strings.Contains(probe, "/tf_static") {
		t.Fatalf("frame probe script missing tf_static topic:\n%s", probe)
	}
	for _, name := range []string{"sensor.toml", "fcu.toml", "p11.toml"} {
		if _, err := os.Stat(filepath.Join(dir, name)); err != nil {
			t.Fatalf("expected generated %s: %v", name, err)
		}
	}
}

func TestValidateP11ConfigFindsOrderedThresholdAndTruthBlockers(t *testing.T) {
	spec := DefaultScanStabilizationSpec()
	spec.InputScanTopic = "/scan"
	spec.OutputScanTopic = "/scan"
	spec.CompensationTiltDeg = 1
	spec.UsesGazeboTruthAsInput = true

	blockers := ValidateP11Config(spec)
	for _, expected := range []string{
		"scan_stabilization_config_invalid: input and output topics must differ",
		"scan_stabilization_config_invalid: tilt thresholds must be ordered",
		"P11 must not use Gazebo truth as input",
	} {
		if !contains(blockers, expected) {
			t.Fatalf("expected blocker %q in %#v", expected, blockers)
		}
	}
}

func TestWriteP12BridgeOverrideRewritesIMUTopic(t *testing.T) {
	path := filepath.Join(t.TempDir(), "bridge.yaml")
	if err := WriteP12BridgeOverride(path, "/imu/raw"); err != nil {
		t.Fatal(err)
	}
	content, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	text := string(content)
	if !strings.Contains(text, `ros_topic_name: "imu/raw"`) {
		t.Fatalf("bridge override did not rewrite imu topic:\n%s", text)
	}
	if strings.Count(text, `ros_topic_name: "imu"`) != 0 {
		t.Fatalf("bridge override still contains original imu topic:\n%s", text)
	}
}

func TestDeepRuntimeHelpersMarkedPortedPartial(t *testing.T) {
	registry := DefaultRegistry()
	for _, id := range []string{
		"sensors",
		"fcu-controller",
		"frame-contract",
		"slam-hover",
		"scan-stabilization",
		"exploration-workflow",
		"scan-robustness-workflow",
	} {
		definition, err := registry.Get(id)
		if err != nil {
			t.Fatal(err)
		}
		if definition.MigrationStatus != "ported_partial" {
			t.Fatalf("%s migration status = %q, want ported_partial", id, definition.MigrationStatus)
		}
	}
}

func assertHasService(t *testing.T, plan ExecutionPlan, name string) {
	t.Helper()
	for _, service := range plan.RuntimeServices {
		if service.ServiceName == name {
			return
		}
	}
	t.Fatalf("execution plan missing service %q", name)
}

func assertHasProbe(t *testing.T, plan ExecutionPlan, name string) {
	t.Helper()
	for _, probe := range plan.ROSProbes {
		if probe.Name == name {
			return
		}
	}
	t.Fatalf("execution plan missing probe %q", name)
}

func assertHasRosbag(t *testing.T, plan ExecutionPlan, name string) {
	t.Helper()
	for _, rosbag := range plan.RosbagRecords {
		if rosbag.Name == name {
			return
		}
	}
	t.Fatalf("execution plan missing rosbag %q", name)
}

func assertHasGate(t *testing.T, plan ExecutionPlan, name string) {
	t.Helper()
	for _, gate := range plan.ResultGates {
		if gate.Name == name {
			return
		}
	}
	t.Fatalf("execution plan missing gate %q", name)
}

func contains(values []string, expected string) bool {
	for _, value := range values {
		if value == expected {
			return true
		}
	}
	return false
}
