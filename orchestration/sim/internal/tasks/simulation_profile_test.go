package tasks

import (
	"strings"
	"testing"

	"navlab/orchestration-sim/internal/config"
	"navlab/orchestration-sim/internal/tasks/helpers"
)

func TestApplySimulationProfileOverridesScanRobustnessRuntimeProfile(t *testing.T) {
	runtimeConfig := config.TaskRuntimeConfig{
		AirframeDisturbance: config.AirframeDisturbanceConfig{
			Profile: "ideal",
		},
		AirframeDisturbanceGate: config.AirframeDisturbanceGateConfig{
			ProfileSet: []string{"ideal", "realistic"},
		},
	}

	updated, err := ApplySimulationProfile(runtimeConfig, Plan{
		TaskID:            "scan-robustness",
		SimulationProfile: "realistic",
	})
	if err != nil {
		t.Fatalf("ApplySimulationProfile() error = %v", err)
	}
	if updated.AirframeDisturbance.Profile != "realistic" {
		t.Fatalf("airframe profile = %q", updated.AirframeDisturbance.Profile)
	}
}

func TestApplySimulationProfileRejectsUnknownScanRobustnessProfile(t *testing.T) {
	runtimeConfig := config.TaskRuntimeConfig{
		AirframeDisturbanceGate: config.AirframeDisturbanceGateConfig{
			ProfileSet: []string{"ideal", "realistic"},
		},
	}

	_, err := ApplySimulationProfile(runtimeConfig, Plan{
		TaskID:            "scan-robustness",
		SimulationProfile: "unsupported_profile",
	})
	if err == nil {
		t.Fatal("ApplySimulationProfile() error = nil, want unknown profile error")
	}
	if !strings.Contains(err.Error(), `simulation profile "unsupported_profile"`) {
		t.Fatalf("error = %v", err)
	}
}

func TestApplySimulationProfileDoesNotChangeOtherTasks(t *testing.T) {
	runtimeConfig := config.TaskRuntimeConfig{
		AirframeDisturbance: config.AirframeDisturbanceConfig{
			Profile: "realistic",
		},
		AirframeDisturbanceGate: config.AirframeDisturbanceGateConfig{
			ProfileSet: []string{"ideal", "realistic"},
		},
	}

	updated, err := ApplySimulationProfile(runtimeConfig, Plan{
		TaskID:            "hover",
		SimulationProfile: "realistic",
	})
	if err != nil {
		t.Fatalf("ApplySimulationProfile() error = %v", err)
	}
	if updated.AirframeDisturbance.Profile != "realistic" {
		t.Fatalf("airframe profile = %q", updated.AirframeDisturbance.Profile)
	}
}

func TestApplySimulationProfileHoverSlamDirectUsesRawSlamOdom(t *testing.T) {
	runtimeConfig := config.TaskRuntimeConfig{
		SlamHover: config.SlamHoverConfig{
			SlamOdomTopic: "/slam/odom",
		},
	}

	updated, err := ApplySimulationProfile(runtimeConfig, Plan{
		TaskID:            "hover",
		SimulationProfile: "slam-direct",
	})
	if err != nil {
		t.Fatalf("ApplySimulationProfile() error = %v", err)
	}
	if updated.SlamHover.ExternalNavInputOdomTopic != "/slam/odom" {
		t.Fatalf("external nav input = %q, want /slam/odom", updated.SlamHover.ExternalNavInputOdomTopic)
	}
}

func TestApplySimulationProfileHoverSlamDirectNoOdomPriorUsesRawSlamOdomAndNoOdomConfig(t *testing.T) {
	runtimeConfig := config.TaskRuntimeConfig{
		SlamHover: config.SlamHoverConfig{
			SlamOdomTopic: "/slam/odom",
		},
	}

	updated, err := ApplySimulationProfile(runtimeConfig, Plan{
		TaskID:            "hover",
		SimulationProfile: "slam-direct-no-odom-prior",
	})
	if err != nil {
		t.Fatalf("ApplySimulationProfile() error = %v", err)
	}
	if updated.SlamHover.ExternalNavInputOdomTopic != "/slam/odom" {
		t.Fatalf("external nav input = %q, want /slam/odom", updated.SlamHover.ExternalNavInputOdomTopic)
	}
	if updated.SlamBackend.CartographerConfigurationBasename != helpers.HoverNoOdomPriorConfigBasename {
		t.Fatalf("cartographer config = %q, want %q", updated.SlamBackend.CartographerConfigurationBasename, helpers.HoverNoOdomPriorConfigBasename)
	}
}

func TestApplySimulationProfileHoverSlamOnlyNoOdomPriorUsesNoOdomConfig(t *testing.T) {
	runtimeConfig := config.TaskRuntimeConfig{
		SlamHover: config.SlamHoverConfig{
			SlamOdomTopic: "/slam/odom",
		},
	}

	updated, err := ApplySimulationProfile(runtimeConfig, Plan{
		TaskID:            "hover-slam-only",
		SimulationProfile: "slam-direct-no-odom-prior",
	})
	if err != nil {
		t.Fatalf("ApplySimulationProfile() error = %v", err)
	}
	if updated.SlamHover.ExternalNavInputOdomTopic != "/slam/odom" {
		t.Fatalf("external nav input = %q, want /slam/odom", updated.SlamHover.ExternalNavInputOdomTopic)
	}
	if updated.SlamBackend.CartographerConfigurationBasename != helpers.HoverNoOdomPriorConfigBasename {
		t.Fatalf("cartographer config = %q, want %q", updated.SlamBackend.CartographerConfigurationBasename, helpers.HoverNoOdomPriorConfigBasename)
	}
}
