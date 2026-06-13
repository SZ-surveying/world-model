package tasks

import (
	"os"
	"path/filepath"
	"strings"
	"testing"

	"navlab/orchestration-sim/internal/config"
	simruntime "navlab/orchestration-sim/internal/runtime"
	"navlab/orchestration-sim/internal/tasks/helpers"
)

func TestBuildRuntimeSpecsFromExecutionPlan(t *testing.T) {
	t.Setenv("NAVLAB_SIM_DISTRO", "")
	project := config.ProjectConfig{
		Orchestration: config.OrchestrationConfig{
			Runtime: config.OrchestrationRuntimeConfig{
				Docker: config.DockerRuntimeConfig{WorkspaceContainerPath: "/workspace"},
			},
		},
		Paths:       config.PathConfig{WorkspaceRoot: "."},
		RosDomainID: "85",
		Navlab: config.NavlabConfig{
			Images: config.ImageCatalog{TagPolicy: "distro-latest"},
		},
		Images: map[string]config.Image{
			"gazebo_sensor":     {Repository: "navlab/gazebo-sensor"},
			"slam":              {Repository: "navlab/slam-cartographer"},
			"official_baseline": {Repository: "navlab/official-baseline"},
		},
	}
	plan := helpers.ExecutionPlan{
		TaskID:      "hover",
		DurationSec: 90,
		RuntimeServices: []helpers.RuntimeServicePlan{
			{
				HelperID:      "sensors",
				ServiceName:   "gazebo_sensor",
				ContainerName: "navlab-official-maze-x2-sensor",
				ImageRef:      "images.gazebo_sensor",
				Network:       "host",
				Command:       []string{"bash", "-lc", "run"},
				Env:           map[string]string{"ROS_DOMAIN_ID": "from config.toml"},
			},
		},
		ROSProbes: []helpers.ROSProbePlan{
			{
				HelperID:     "sensors",
				Name:         "rangefinder_probe",
				ScriptPath:   "rangefinder_probe.py",
				OutputPath:   "rangefinder_probe.txt",
				RuntimeImage: "images.runtime",
				Topics:       []string{"/navlab/rangefinder/range"},
			},
		},
		RosbagRecords: []helpers.RosbagRecordPlan{
			{
				HelperID:  "slam-hover",
				Name:      "hover_rosbag",
				OutputDir: "rosbag",
				Topics:    []string{"/tf", "/scan"},
			},
		},
	}

	bundle, err := BuildRuntimeSpecs(project, plan, t.TempDir())
	if err != nil {
		t.Fatal(err)
	}
	if len(bundle.Services) != 2 {
		t.Fatalf("services = %#v", bundle.Services)
	}
	official := serviceByName(bundle.Services, "official_baseline")
	if official == nil || official.Image != "navlab/official-baseline:humble-latest" {
		t.Fatalf("official baseline service = %#v", bundle.Services)
	}
	sensor := serviceByName(bundle.Services, "gazebo_sensor")
	if sensor == nil || sensor.Image != "navlab/gazebo-sensor:humble-latest" {
		t.Fatalf("gazebo sensor service = %#v", bundle.Services)
	}
	if sensor.Env["ROS_DOMAIN_ID"] != "85" {
		t.Fatalf("service env = %#v", sensor.Env)
	}
	if len(bundle.Probes) != 1 || bundle.Probes[0].Image != "navlab/official-baseline:humble-latest" {
		t.Fatalf("probes = %#v", bundle.Probes)
	}
	if len(bundle.Rosbags) != 1 {
		t.Fatalf("rosbags = %#v", bundle.Rosbags)
	}
	if filepath.Base(bundle.Rosbags[0].TopicsProfile) != "hover_rosbag.txt" {
		t.Fatalf("topics profile = %s", bundle.Rosbags[0].TopicsProfile)
	}
}

func TestBuildRuntimeSpecsMapsWorkspaceArtifactsToContainerWorkspace(t *testing.T) {
	t.Setenv("NAVLAB_SIM_DISTRO", "")
	workspace := t.TempDir()
	artifactDir := filepath.Join(workspace, "artifacts", "sim", "hover", "run-1")
	project := config.ProjectConfig{
		Orchestration: config.OrchestrationConfig{
			Runtime: config.OrchestrationRuntimeConfig{
				Docker: config.DockerRuntimeConfig{WorkspaceContainerPath: "/workspace"},
			},
		},
		Paths:       config.PathConfig{WorkspaceRoot: workspace},
		RosDomainID: "85",
		Navlab: config.NavlabConfig{
			Images: config.ImageCatalog{TagPolicy: "distro-latest"},
		},
		Images: map[string]config.Image{
			"official_baseline": {Repository: "navlab/official-baseline"},
		},
	}
	plan := helpers.ExecutionPlan{
		TaskID:      "hover",
		DurationSec: 90,
		RuntimeServices: []helpers.RuntimeServicePlan{
			{
				HelperID:      "sensors",
				ServiceName:   "gazebo_sensor",
				ContainerName: "sensor",
				ImageRef:      "images.official_baseline",
				Command:       []string{"bash", "-lc", "python3 artifacts/fcu_controller_runtime.py --log artifacts/gazebo_sensor.log"},
				Env:           map[string]string{"NAVLAB_CONFIG": "artifacts/gazebo_sensor_runtime.toml"},
			},
		},
		ROSProbes: []helpers.ROSProbePlan{
			{
				HelperID:     "sensors",
				Name:         "rangefinder_probe",
				ScriptPath:   "rangefinder_probe.py",
				OutputPath:   "rangefinder_probe.txt",
				RuntimeImage: "images.runtime",
				Topics:       []string{"/navlab/rangefinder/range"},
			},
		},
		RosbagRecords: []helpers.RosbagRecordPlan{
			{Name: "hover_rosbag", OutputDir: "rosbag", Topics: []string{"/tf"}},
		},
	}

	bundle, err := BuildRuntimeSpecs(project, plan, artifactDir)
	if err != nil {
		t.Fatal(err)
	}
	sensor := serviceByName(bundle.Services, "gazebo_sensor")
	if sensor == nil {
		t.Fatalf("services = %#v", bundle.Services)
		return
	}
	serviceCommand := strings.Join(sensor.Command, " ")
	if !strings.Contains(serviceCommand, "/workspace/artifacts/sim/hover/run-1/fcu_controller_runtime.py") {
		t.Fatalf("service command = %q", serviceCommand)
	}
	if sensor.Env["NAVLAB_CONFIG"] != "/workspace/artifacts/sim/hover/run-1/gazebo_sensor_runtime.toml" {
		t.Fatalf("service env = %#v", sensor.Env)
	}
	probeCommand := strings.Join(bundle.Probes[0].Command, " ")
	if !strings.Contains(probeCommand, "/workspace/artifacts/sim/hover/run-1/rangefinder_probe.py") {
		t.Fatalf("probe command = %q", probeCommand)
	}
	if bundle.Rosbags[0].OutputPath != "/workspace/artifacts/sim/hover/run-1/rosbag" {
		t.Fatalf("rosbag output = %q", bundle.Rosbags[0].OutputPath)
	}
	if bundle.Probes[0].Volumes[0].Source != workspace {
		t.Fatalf("volume source = %q, want %q", bundle.Probes[0].Volumes[0].Source, workspace)
	}
}

func TestBuildRuntimePlanContract(t *testing.T) {
	artifactDir := t.TempDir()
	topicsProfile := filepath.Join(artifactDir, "hover_rosbag.txt")
	if err := os.WriteFile(topicsProfile, []byte("/tf\n/scan\n"), 0o644); err != nil {
		t.Fatal(err)
	}

	contract, err := BuildRuntimePlanContract(
		Plan{TaskID: "hover"},
		"20260613T010203Z",
		RuntimeSpecBundle{
			Services: []simruntime.ServiceSpec{
				{
					Name:        "gazebo_sensor",
					ServiceRole: "sensors",
					Image:       "navlab/gazebo-sensor:latest",
					Command:     []string{"bash", "-lc", "run-sensor"},
					Env:         map[string]string{"ROS_DOMAIN_ID": "85"},
					CWD:         "/workspace",
					Volumes: []simruntime.VolumeMount{
						{Source: "/host/ws", Target: "/workspace", Mode: "ro"},
					},
					Networks: []string{"host"},
					Required: true,
					LogPath:  filepath.Join(artifactDir, "gazebo_sensor.start.log"),
				},
			},
			Probes: []simruntime.ProbeSpec{
				{
					Name:        "rangefinder_probe",
					ServiceRole: "sensors",
					Image:       "navlab/official-baseline:latest",
					Command:     []string{"bash", "-lc", "probe"},
					Env:         map[string]string{"ROS_DOMAIN_ID": "85"},
					TimeoutSec:  30,
					Required:    true,
					LogPath:     filepath.Join(artifactDir, "rangefinder_probe.log"),
				},
			},
			Rosbags: []simruntime.RosbagSpec{
				{
					Name:          "hover_rosbag",
					ServiceRole:   "slam-hover",
					TopicsProfile: topicsProfile,
					OutputPath:    "/workspace/artifacts/rosbag",
					DurationSec:   90,
					Storage:       "mcap",
					Required:      true,
					LogPath:       filepath.Join(artifactDir, "hover_rosbag.log"),
				},
			},
		},
	)
	if err != nil {
		t.Fatal(err)
	}
	if contract["schemaVersion"] != "navlab.runtime.runtime_plan.v1" {
		t.Fatalf("schemaVersion = %#v", contract["schemaVersion"])
	}
	services := contract["services"].([]map[string]any)
	if services[0]["backend"] != "RUNTIME_BACKEND_DOCKER" || services[0]["role"] != "sensors" {
		t.Fatalf("service contract = %#v", services[0])
	}
	volumes := services[0]["volumes"].([]map[string]any)
	if volumes[0]["readOnly"] != true {
		t.Fatalf("volume contract = %#v", volumes[0])
	}
	probes := contract["probes"].([]map[string]any)
	if probes[0]["timeoutSec"] != float64(30) {
		t.Fatalf("probe contract = %#v", probes[0])
	}
	rosbags := contract["rosbags"].([]map[string]any)
	topics := rosbags[0]["topics"].([]string)
	if strings.Join(topics, ",") != "/tf,/scan" {
		t.Fatalf("rosbag topics = %#v", topics)
	}
}

func serviceByName(services []simruntime.ServiceSpec, name string) *simruntime.ServiceSpec {
	for index := range services {
		if services[index].Name == name {
			return &services[index]
		}
	}
	return nil
}

func TestBuildRuntimeSpecsRejectsMissingImage(t *testing.T) {
	_, err := BuildRuntimeSpecs(
		config.ProjectConfig{},
		helpers.ExecutionPlan{
			RuntimeServices: []helpers.RuntimeServicePlan{
				{HelperID: "slam", ServiceName: "slam_backend", ImageRef: "images.slam", Command: []string{"true"}},
			},
		},
		t.TempDir(),
	)
	if err == nil {
		t.Fatal("BuildRuntimeSpecs error = nil, want missing image error")
	}
}
