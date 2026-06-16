package config

import (
	"fmt"

	mapstructure "github.com/go-viper/mapstructure/v2"
)

type TaskRuntimeConfig struct {
	TaskID                  string                        `json:"task_id"`
	OfficialMazeX2          OfficialMazeX2Config          `json:"official_maze_x2"`
	RangefinderIMU          RangefinderIMUConfig          `json:"rangefinder_imu"`
	SlamBackend             SlamBackendConfig             `json:"slam_backend"`
	FCUController           FCUControllerConfig           `json:"fcu_controller"`
	FrameContract           FrameContractConfig           `json:"frame_contract"`
	SlamHover               SlamHoverConfig               `json:"slam_hover"`
	MotionGate              MotionGateConfig              `json:"motion_gate"`
	ExplorationGate         ExplorationGateConfig         `json:"exploration_gate"`
	Nav2                    Nav2Config                    `json:"nav2"`
	NavigationAdapter       NavigationAdapterConfig       `json:"navigation_adapter"`
	NavigationMission       NavigationMissionConfig       `json:"navigation_mission"`
	ScanIntegrityGate       ScanIntegrityGateConfig       `json:"scan_integrity_gate"`
	ScanStabilization       ScanStabilizationConfig       `json:"scan_stabilization"`
	ScanStabilizationGate   ScanStabilizationGateConfig   `json:"scan_stabilization_gate"`
	AirframeDisturbance     AirframeDisturbanceConfig     `json:"airframe_disturbance"`
	AirframeDisturbanceGate AirframeDisturbanceGateConfig `json:"airframe_disturbance_gate"`
	Landing                 LandingConfig                 `json:"landing"`
	ScanRobustness          ScanRobustnessTaskConfig      `json:"scan_robustness"`
}

type ScanRobustnessTaskConfig struct {
	Live         bool     `json:"live" mapstructure:"live"`
	LiveProfiles []string `json:"live_profiles" mapstructure:"live_profiles"`
}

func BuildTaskRuntimeConfig(project ProjectConfig, task TaskConfig) (TaskRuntimeConfig, error) {
	runtimeConfig := TaskRuntimeConfig{
		TaskID:                  task.ID,
		OfficialMazeX2:          project.OfficialMazeX2,
		RangefinderIMU:          project.RangefinderIMU,
		SlamBackend:             project.SlamBackend,
		FCUController:           project.FCUController,
		FrameContract:           project.FrameContract,
		SlamHover:               project.SlamHover,
		MotionGate:              project.MotionGate,
		ExplorationGate:         project.ExplorationGate,
		Nav2:                    project.Nav2,
		NavigationAdapter:       project.NavigationAdapter,
		NavigationMission:       project.NavigationMission,
		ScanIntegrityGate:       project.ScanIntegrityGate,
		ScanStabilization:       project.ScanStabilization,
		ScanStabilizationGate:   project.ScanStabilizationGate,
		AirframeDisturbance:     project.AirframeDisturbance,
		AirframeDisturbanceGate: project.AirframeDisturbanceGate,
		Landing:                 project.Landing,
		ScanRobustness:          ScanRobustnessTaskConfig{Live: true},
	}
	if err := applySection(task.Sections, "official_maze_x2", &runtimeConfig.OfficialMazeX2); err != nil {
		return runtimeConfig, err
	}
	if err := applySection(task.Sections, "rangefinder_imu", &runtimeConfig.RangefinderIMU); err != nil {
		return runtimeConfig, err
	}
	if err := applySection(task.Sections, "slam_backend", &runtimeConfig.SlamBackend); err != nil {
		return runtimeConfig, err
	}
	if err := applySection(task.Sections, "fcu_controller", &runtimeConfig.FCUController); err != nil {
		return runtimeConfig, err
	}
	if err := applySection(task.Sections, "frame_contract", &runtimeConfig.FrameContract); err != nil {
		return runtimeConfig, err
	}
	if err := applySection(task.Sections, "slam_hover", &runtimeConfig.SlamHover); err != nil {
		return runtimeConfig, err
	}
	if err := applySection(task.Sections, "motion_gate", &runtimeConfig.MotionGate); err != nil {
		return runtimeConfig, err
	}
	if err := applySection(task.Sections, "exploration_gate", &runtimeConfig.ExplorationGate); err != nil {
		return runtimeConfig, err
	}
	if err := applySection(task.Sections, "nav2", &runtimeConfig.Nav2); err != nil {
		return runtimeConfig, err
	}
	if err := applySection(task.Sections, "navigation_adapter", &runtimeConfig.NavigationAdapter); err != nil {
		return runtimeConfig, err
	}
	if err := applySection(task.Sections, "navigation_mission", &runtimeConfig.NavigationMission); err != nil {
		return runtimeConfig, err
	}
	if err := applySection(task.Sections, "scan_integrity_gate", &runtimeConfig.ScanIntegrityGate); err != nil {
		return runtimeConfig, err
	}
	if err := applySection(task.Sections, "scan_stabilization", &runtimeConfig.ScanStabilization); err != nil {
		return runtimeConfig, err
	}
	if err := applySection(task.Sections, "scan_stabilization_gate", &runtimeConfig.ScanStabilizationGate); err != nil {
		return runtimeConfig, err
	}
	if err := applySection(task.Sections, "airframe_disturbance", &runtimeConfig.AirframeDisturbance); err != nil {
		return runtimeConfig, err
	}
	if err := applySection(task.Sections, "airframe_disturbance_gate", &runtimeConfig.AirframeDisturbanceGate); err != nil {
		return runtimeConfig, err
	}
	if err := applySection(task.Sections, "landing", &runtimeConfig.Landing); err != nil {
		return runtimeConfig, err
	}
	if err := applySection(task.Sections, "scan_robustness", &runtimeConfig.ScanRobustness); err != nil {
		return runtimeConfig, err
	}
	return runtimeConfig, nil
}

func applySection[T any](sections map[string]any, name string, target *T) error {
	raw, ok := sections[name]
	if !ok {
		return nil
	}
	decoder, err := mapstructure.NewDecoder(&mapstructure.DecoderConfig{
		Result:           target,
		TagName:          "mapstructure",
		WeaklyTypedInput: true,
		ZeroFields:       false,
	})
	if err != nil {
		return err
	}
	if err := decoder.Decode(raw); err != nil {
		return fmt.Errorf("%s: %w", name, err)
	}
	return nil
}
