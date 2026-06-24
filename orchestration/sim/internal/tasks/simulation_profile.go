package tasks

import (
	"fmt"

	"navlab/orchestration-sim/internal/config"
	"navlab/orchestration-sim/internal/tasks/helpers"
)

func ApplySimulationProfile(runtimeConfig config.TaskRuntimeConfig, plan Plan) (config.TaskRuntimeConfig, error) {
	if isHoverSLAMProfileTask(plan.TaskID) && plan.SimulationProfile == "slam-direct" {
		runtimeConfig.SlamHover.ExternalNavInputOdomTopic = runtimeConfig.SlamHover.SlamOdomTopic
		return runtimeConfig, nil
	}
	if isHoverSLAMProfileTask(plan.TaskID) && plan.SimulationProfile == "slam-direct-no-odom-prior" {
		runtimeConfig.SlamHover.ExternalNavInputOdomTopic = runtimeConfig.SlamHover.SlamOdomTopic
		runtimeConfig.SlamBackend.CartographerConfigurationBasename = helpers.HoverNoOdomPriorConfigBasename
		return runtimeConfig, nil
	}
	if plan.TaskID != "scan-robustness" || plan.SimulationProfile == "" {
		return runtimeConfig, nil
	}
	if !stringInSlice(plan.SimulationProfile, runtimeConfig.AirframeDisturbanceGate.ProfileSet) {
		return runtimeConfig, fmt.Errorf("simulation profile %q is not in airframe disturbance profile_set %v", plan.SimulationProfile, runtimeConfig.AirframeDisturbanceGate.ProfileSet)
	}
	runtimeConfig.AirframeDisturbance.Profile = plan.SimulationProfile
	return runtimeConfig, nil
}

func isHoverSLAMProfileTask(taskID string) bool {
	switch taskID {
	case "hover", "hover-slam-only":
		return true
	default:
		return false
	}
}

func stringInSlice(value string, values []string) bool {
	for _, candidate := range values {
		if candidate == value {
			return true
		}
	}
	return false
}
