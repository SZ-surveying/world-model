package tasks

import (
	"os"
	"path/filepath"
	"reflect"
	"strings"
	"testing"

	"github.com/pelletier/go-toml/v2"

	"navlab/orchestration-sim/internal/config"
	"navlab/orchestration-sim/internal/tasks/helpers"
)

func TestGenerateRuntimeArtifactsFromConfiguredTasks(t *testing.T) {
	t.Setenv("NAVLAB_SIM_OVERLAY_SOURCE_MODE", "fixture")
	tests := []struct {
		name      string
		taskID    string
		wantFiles []string
	}{
		{
			name:   "hover",
			taskID: "hover",
			wantFiles: []string{
				"official_maze_overlay_runtime.py",
				"bridge_override.yaml",
				"vendor_profile.yaml",
				"model_overlay.sdf",
				"gazebo-iris-rangefinder.parm",
				"gazebo_sensor_runtime.toml",
				"rangefinder_probe.py",
				"imu_probe.py",
				"slam_runtime.toml",
				"external_nav_bridge_params.yaml",
				"frame_contract_runtime.toml",
				"frame_contract_probe.py",
				"slam_hover_runtime.toml",
				"hover_mission_runtime.py",
				"slam_hover_probe.py",
			},
		},
		{
			name:   "exploration",
			taskID: "exploration",
			wantFiles: []string{
				"official_maze_overlay_runtime.py",
				"exploration_runtime.toml",
				"exploration_workflow_runtime.py",
				"exploration_probe.py",
			},
		},
		{
			name:   "navigation",
			taskID: "navigation",
			wantFiles: []string{
				"official_maze_overlay_runtime.py",
				"nav2_params.yaml",
				"navigation_adapter_runtime.toml",
				"navigation_foxglove_lite_profile.json",
				"navigation_adapter_runtime.py",
				"navigation_mission_runtime.py",
				"nav2_lifecycle_probe.py",
				"costmap_health_probe.py",
				"navigation_status_probe.py",
			},
		},
		{
			name:   "scan-robustness",
			taskID: "scan-robustness",
			wantFiles: []string{
				"official_maze_overlay_runtime.py",
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
			if tt.taskID == "hover" {
				assertParamOverlayContainsExternalNavAndRangefinder(t, filepath.Join(artifactDir, "gazebo-iris-rangefinder.parm"))
				assertSensorRuntimeUsesBenewakeSerial(t, filepath.Join(artifactDir, "gazebo_sensor_runtime.toml"))
			}
		})
	}
}

func assertSensorRuntimeUsesBenewakeSerial(t *testing.T, path string) {
	t.Helper()
	data, err := os.ReadFile(path)
	if err != nil {
		t.Fatalf("read generated sensor runtime config: %v", err)
	}
	text := string(data)
	for _, want := range []string{
		`virtual_serial_link = '/tmp/navlab_benewake_tfmini'`,
		"serial_baud = 115200",
	} {
		if !strings.Contains(text, want) {
			t.Fatalf("generated sensor runtime config missing %q:\n%s", want, text)
		}
	}
	for _, stale := range []string{
		"endpoint =",
		"mavlink_orientation",
		"source_system",
		"source_component",
		"sensor_id",
		"covariance_cm",
	} {
		if strings.Contains(text, stale) {
			t.Fatalf("generated sensor runtime config retained MAVLink sender field %q:\n%s", stale, text)
		}
	}
}

func assertParamOverlayContainsExternalNavAndRangefinder(t *testing.T, path string) {
	t.Helper()
	data, err := os.ReadFile(path)
	if err != nil {
		t.Fatalf("read generated param overlay: %v", err)
	}
	text := string(data)
	for _, want := range []string{
		"GPS1_TYPE 0",
		"VISO_TYPE 1",
		"EK3_SRC1_POSXY 6",
		"EK3_SRC1_POSZ 2",
		"EK3_SRC1_VELXY 6",
		"EK3_SRC1_VELZ 6",
		"EK3_SRC1_YAW 6",
		"SERIAL7_PROTOCOL 9",
		"RNGFND1_PIN -1",
		"RNGFND1_SCALING 3",
		"RNGFND1_TYPE 20",
		"RNGFND1_MIN_CM 10",
		"RNGFND1_MAX_CM 1200",
		"RNGFND1_GNDCLEAR 15",
	} {
		if !strings.Contains(text, want) {
			t.Fatalf("generated param overlay missing %q:\n%s", want, text)
		}
	}
	for _, stale := range []string{
		"RNGFND1_TYPE 1",
		"RNGFND1_TYPE 10",
		"RNGFND1_MIN_CM 5",
		"RNGFND1_MAX_CM 600",
		"RNGFND1_GNDCLEAR 10",
		"RNGFND1_SCALING 10",
		"RNGFND1_PIN 0",
		"RNGFND1_MAX 50.00",
		"SIM_SONAR_SCALE",
	} {
		if strings.Contains(text, stale) {
			t.Fatalf("generated param overlay should not override ExternalNav profile with %q:\n%s", stale, text)
		}
	}
}

func TestNavigationAdapterRuntimeScriptUsesIntentTopicNotDirectFCU(t *testing.T) {
	loader := config.NewLoader("../../config.toml")
	project, err := loader.LoadProject()
	if err != nil {
		t.Fatalf("LoadProject() error = %v", err)
	}
	taskConfig, err := loader.LoadTask(project, "navigation")
	if err != nil {
		t.Fatalf("LoadTask(navigation) error = %v", err)
	}
	runtimeConfig, err := config.BuildTaskRuntimeConfig(project, taskConfig)
	if err != nil {
		t.Fatalf("BuildTaskRuntimeConfig(navigation) error = %v", err)
	}
	task, err := DefaultRegistry().ConfigureOne(taskConfig)
	if err != nil {
		t.Fatalf("ConfigureOne(navigation) error = %v", err)
	}
	plan, err := task.Plan(PlanOptions{}, helpers.DefaultRegistry())
	if err != nil {
		t.Fatalf("Plan(navigation) error = %v", err)
	}

	artifactDir := t.TempDir()
	if _, err := GenerateRuntimeArtifacts(project, plan, runtimeConfig, artifactDir); err != nil {
		t.Fatalf("GenerateRuntimeArtifacts(navigation) error = %v", err)
	}
	script, err := os.ReadFile(filepath.Join(artifactDir, "navigation_adapter_runtime.py"))
	if err != nil {
		t.Fatalf("ReadFile(navigation_adapter_runtime.py) error = %v", err)
	}
	text := string(script)
	if !strings.Contains(text, "SetpointIntentTopic") || !strings.Contains(text, "create_subscription") {
		t.Fatalf("navigation adapter script missing intent/subscription behavior:\n%s", text)
	}
	if strings.Contains(text, `"/ap/v1/cmd_vel"`) {
		t.Fatalf("navigation adapter script directly references FCU cmd_vel:\n%s", text)
	}
	missionScript, err := os.ReadFile(filepath.Join(artifactDir, "navigation_mission_runtime.py"))
	if err != nil {
		t.Fatalf("ReadFile(navigation_mission_runtime.py) error = %v", err)
	}
	missionText := string(missionScript)
	if !strings.Contains(missionText, "NavigateToPose") || !strings.Contains(missionText, "nav2_action_unavailable") {
		t.Fatalf("navigation mission script missing NavigateToPose readiness behavior:\n%s", missionText)
	}
	for _, expected := range []string{"mission_goals", "ExitGoal", "completion_policy", "frontier_candidate", "coverage_growth", "blacklisted_goals", "unreachable_blacklisted", "map_ready", "costmap_ready"} {
		if !strings.Contains(missionText, expected) {
			t.Fatalf("navigation mission script missing %q:\n%s", expected, missionText)
		}
	}
	profile, err := os.ReadFile(filepath.Join(artifactDir, "navigation_foxglove_lite_profile.json"))
	if err != nil {
		t.Fatalf("ReadFile(navigation_foxglove_lite_profile.json) error = %v", err)
	}
	profileText := string(profile)
	for _, expected := range []string{"\"role\": \"map\"", "\"role\": \"path\"", "\"role\": \"global_costmap\"", "\"role\": \"scan\"", "\"role\": \"trajectory\"", "\"role\": \"goals\"", "\"role\": \"events\""} {
		if !strings.Contains(profileText, expected) {
			t.Fatalf("navigation Foxglove profile missing %q:\n%s", expected, profileText)
		}
	}
}

func TestFCUControllerRuntimeScriptKeepsSubscriptionsAlive(t *testing.T) {
	loader := config.NewLoader("../../config.toml")
	project, err := loader.LoadProject()
	if err != nil {
		t.Fatalf("LoadProject() error = %v", err)
	}
	taskConfig, err := loader.LoadTask(project, "exploration")
	if err != nil {
		t.Fatalf("LoadTask(exploration) error = %v", err)
	}
	runtimeConfig, err := config.BuildTaskRuntimeConfig(project, taskConfig)
	if err != nil {
		t.Fatalf("BuildTaskRuntimeConfig(exploration) error = %v", err)
	}
	task, err := DefaultRegistry().ConfigureOne(taskConfig)
	if err != nil {
		t.Fatalf("ConfigureOne(exploration) error = %v", err)
	}
	plan, err := task.Plan(PlanOptions{}, helpers.DefaultRegistry())
	if err != nil {
		t.Fatalf("Plan(exploration) error = %v", err)
	}

	artifactDir := t.TempDir()
	if _, err := GenerateRuntimeArtifacts(project, plan, runtimeConfig, artifactDir); err != nil {
		t.Fatalf("GenerateRuntimeArtifacts(exploration) error = %v", err)
	}
	script, err := os.ReadFile(filepath.Join(artifactDir, "fcu_controller_runtime.py"))
	if err != nil {
		t.Fatalf("ReadFile(fcu_controller_runtime.py) error = %v", err)
	}
	text := string(script)
	for _, expected := range []string{
		"subscriptions = []",
		`subscriptions.append(node.create_subscription(PoseStamped, SPEC["pose_topic"], on_pose, qos_profile_sensor_data))`,
		`subscriptions.append(node.create_subscription(Odometry, SPEC["slam_odom_topic"], on_slam_odom, qos_profile_sensor_data))`,
		`subscriptions.append(node.create_subscription(String, SPEC["setpoint_intent_topic"], on_setpoint_intent, 10))`,
		"import threading",
		`state["pose_source"] = "slam_odom"`,
		`state.get("pose_source") != "slam_odom"`,
		"bootstrap_thread = threading.Thread(target=run_bootstrap, daemon=True)",
		"bootstrap_thread.start()",
		"def bootstrap_ready(state: dict) -> bool:",
		`and (bootstrap.get("takeoff") or {}).get("ok", False)`,
		`takeoff_min_height_m`,
		`takeoff_min_height_ratio`,
		`target_min = max(min_height_m, altitude_m * min_height_ratio)`,
		`def send_mavlink_local_position_setpoint(payload: dict) -> None:`,
		`master.mav.set_position_target_local_ned_send`,
		`mavutil.mavlink.MAV_FRAME_LOCAL_NED`,
		`mavlink_setpoint_count`,
		`refresh_mavlink_local_position(master)`,
		`setpoint_lookahead_sec`,
	} {
		if !strings.Contains(text, expected) {
			t.Fatalf("fcu controller script missing %q:\n%s", expected, text)
		}
	}
	if strings.Contains(text, `target_min = max(0.2, altitude_m * 0.45)`) {
		t.Fatalf("fcu controller script kept stale hard-coded takeoff threshold:\n%s", text)
	}
	if strings.Contains(text, `"state": "hover_hold" if ready`) {
		t.Fatalf("fcu controller must not alias controller readiness to hover_hold:\n%s", text)
	}
}

func TestHoverRuntimeScriptCallsPythonMissionRuntime(t *testing.T) {
	loader := config.NewLoader("../../config.toml")
	project, err := loader.LoadProject()
	if err != nil {
		t.Fatalf("LoadProject() error = %v", err)
	}
	taskConfig, err := loader.LoadTask(project, "hover")
	if err != nil {
		t.Fatalf("LoadTask(hover) error = %v", err)
	}
	runtimeConfig, err := config.BuildTaskRuntimeConfig(project, taskConfig)
	if err != nil {
		t.Fatalf("BuildTaskRuntimeConfig(hover) error = %v", err)
	}
	task, err := DefaultRegistry().ConfigureOne(taskConfig)
	if err != nil {
		t.Fatalf("ConfigureOne(hover) error = %v", err)
	}
	plan, err := task.Plan(PlanOptions{}, helpers.DefaultRegistry())
	if err != nil {
		t.Fatalf("Plan(hover) error = %v", err)
	}

	artifactDir := t.TempDir()
	if _, err := GenerateRuntimeArtifacts(project, plan, runtimeConfig, artifactDir); err != nil {
		t.Fatalf("GenerateRuntimeArtifacts(hover) error = %v", err)
	}
	script, err := os.ReadFile(filepath.Join(artifactDir, "hover_mission_runtime.py"))
	if err != nil {
		t.Fatalf("ReadFile(hover_mission_runtime.py) error = %v", err)
	}
	text := string(script)
	for _, expected := range []string{
		"from navlab.sim.companion.nodes.hover_mission import run",
		"mission_summary.json",
		"takeoff-alt-m",
		"hover-hold-sec",
		"landing-status-topic",
		"require-external-nav",
	} {
		if !strings.Contains(text, expected) {
			t.Fatalf("hover mission wrapper missing %q:\n%s", expected, text)
		}
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
