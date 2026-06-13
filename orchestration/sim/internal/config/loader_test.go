package config

import (
	"path/filepath"
	"testing"
)

func TestLoaderReadsProjectAndYAMLTasks(t *testing.T) {
	configPath := filepath.Join("..", "..", "config.toml")
	loader := NewLoader(configPath)

	project, err := loader.LoadProject()
	if err != nil {
		t.Fatalf("LoadProject() error = %v", err)
	}
	if project.Orchestration.Family != "sim" {
		t.Fatalf("family = %q, want sim", project.Orchestration.Family)
	}
	if project.Runtime.Mode != "simulation" || project.Orchestration.Runtime.Backend != "docker" {
		t.Fatalf("runtime = %#v orchestration.runtime = %#v, want simulation/docker", project.Runtime, project.Orchestration.Runtime)
	}
	if project.Orchestration.Runtime.Docker.WorkspaceContainerPath != "/workspace" {
		t.Fatalf("workspace container path = %q, want /workspace", project.Orchestration.Runtime.Docker.WorkspaceContainerPath)
	}
	if project.Router.TCPPort != "5760" {
		t.Fatalf("router tcp port = %q, want 5760", project.Router.TCPPort)
	}
	if project.SessionID != "navlab_companion_sitl_gazebo" {
		t.Fatalf("session id = %q, want navlab_companion_sitl_gazebo", project.SessionID)
	}
	if project.Landing.ExplorationPolicy != "return_home_then_land" {
		t.Fatalf("exploration landing policy = %q, want return_home_then_land", project.Landing.ExplorationPolicy)
	}
	if project.SITL.Image == "" || len(project.SITL.ExtraArgs) == 0 {
		t.Fatalf("sitl config not loaded: %#v", project.SITL)
	}
	if project.Slam.Backend != "cartographer" {
		t.Fatalf("slam backend = %q, want cartographer", project.Slam.Backend)
	}
	if project.Official.ExternalNavRoute != "official_dds" {
		t.Fatalf("official external nav route = %q, want official_dds", project.Official.ExternalNavRoute)
	}
	if project.RangefinderIMU.RangefinderRangeTopic != "/rangefinder/down/range" {
		t.Fatalf("rangefinder range topic = %q", project.RangefinderIMU.RangefinderRangeTopic)
	}
	if project.SlamBackend.SlamOdomTopic != "/slam/odom" {
		t.Fatalf("slam odom topic = %q", project.SlamBackend.SlamOdomTopic)
	}
	if project.FCUController.CmdVelTopic != "/ap/v1/cmd_vel" {
		t.Fatalf("fcu cmd_vel topic = %q", project.FCUController.CmdVelTopic)
	}
	if project.FrameContract.MapFrameID != "map" || len(project.FrameContract.RequiredFrames) == 0 {
		t.Fatalf("frame contract = %#v", project.FrameContract)
	}
	if project.SlamHover.HoverStatusTopic != "/navlab/hover/status" {
		t.Fatalf("hover status topic = %q", project.SlamHover.HoverStatusTopic)
	}
	if project.MotionGate.MotionStatusTopic != "/navlab/motion/status" {
		t.Fatalf("motion status topic = %q", project.MotionGate.MotionStatusTopic)
	}
	if project.ExplorationGate.ExplorationStatusTopic != "/navlab/exploration/status" {
		t.Fatalf("exploration status topic = %q", project.ExplorationGate.ExplorationStatusTopic)
	}
	if project.ScanStabilization.Mode != "bounded_2d_projection" || !project.ScanStabilization.Enabled {
		t.Fatalf("scan stabilization = %#v", project.ScanStabilization)
	}
	if project.AirframeDisturbance.Profile != "realistic" {
		t.Fatalf("airframe disturbance profile = %q", project.AirframeDisturbance.Profile)
	}
	if len(project.AirframeDisturbanceGate.RequiredProfiles) != 2 {
		t.Fatalf("airframe required profiles = %#v", project.AirframeDisturbanceGate.RequiredProfiles)
	}
	if project.Navlab.Images.TagPolicy != "distro-git-commit" {
		t.Fatalf("image tag policy = %q, want distro-git-commit", project.Navlab.Images.TagPolicy)
	}
	if project.Navlab.Images.Distro != "humble" {
		t.Fatalf("image distro = %q, want humble", project.Navlab.Images.Distro)
	}
	if project.Images["ros_base"].Repository != "navlab/ros-base" {
		t.Fatalf("ros base image = %#v", project.Images["ros_base"])
	}
	if project.Images["official_baseline"].Repository != "navlab/official-baseline" {
		t.Fatalf("official baseline image = %#v", project.Images["official_baseline"])
	}

	tasks, err := loader.LoadTasks(project)
	if err != nil {
		t.Fatalf("LoadTasks() error = %v", err)
	}
	if len(tasks) != 3 {
		t.Fatalf("len(tasks) = %d, want 3", len(tasks))
	}
	if tasks[0].ID != "exploration" {
		t.Fatalf("first task = %q, want exploration", tasks[0].ID)
	}
}

func TestLoaderRejectsUnknownTask(t *testing.T) {
	configPath := filepath.Join("..", "..", "config.toml")
	loader := NewLoader(configPath)
	project, err := loader.LoadProject()
	if err != nil {
		t.Fatalf("LoadProject() error = %v", err)
	}

	if _, err := loader.LoadTask(project, "preflight"); err == nil {
		t.Fatal("LoadTask(preflight) error = nil, want error")
	}
}
