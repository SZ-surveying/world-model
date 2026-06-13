package tasks

import (
	"fmt"

	"navlab/orchestration-sim/internal/config"
)

func ApplySimulationProfile(runtimeConfig config.TaskRuntimeConfig, plan Plan) (config.TaskRuntimeConfig, error) {
	if plan.TaskID != "scan-robustness" || plan.SimulationProfile == "" {
		return runtimeConfig, nil
	}
	if !stringInSlice(plan.SimulationProfile, runtimeConfig.AirframeDisturbanceGate.ProfileSet) {
		return runtimeConfig, fmt.Errorf("simulation profile %q is not in airframe disturbance profile_set %v", plan.SimulationProfile, runtimeConfig.AirframeDisturbanceGate.ProfileSet)
	}
	runtimeConfig.AirframeDisturbance.Profile = plan.SimulationProfile
	return runtimeConfig, nil
}

func stringInSlice(value string, values []string) bool {
	for _, candidate := range values {
		if candidate == value {
			return true
		}
	}
	return false
}
