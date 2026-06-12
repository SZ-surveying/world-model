package tasks

import (
	"os"
	"path/filepath"
	"strconv"

	"navlab/orchestration-sim/internal/config"
	"navlab/orchestration-sim/internal/tasks/helpers"
)

type GeneratedRuntimeArtifact struct {
	Type string `json:"type"`
	Path string `json:"path"`
}

func GenerateRuntimeArtifacts(
	project config.ProjectConfig,
	plan Plan,
	runtimeConfig config.TaskRuntimeConfig,
	artifactDir string,
) ([]GeneratedRuntimeArtifact, error) {
	var generated []GeneratedRuntimeArtifact
	if err := os.MkdirAll(artifactDir, 0o755); err != nil {
		return nil, err
	}
	if hasHelper(plan, "navlab-models") {
		bridge := filepath.Join(artifactDir, "bridge_override.yaml")
		if err := helpers.WriteP1BridgeOverride(bridge); err != nil {
			return nil, err
		}
		generated = append(generated, GeneratedRuntimeArtifact{Type: "bridge_override", Path: bridge})
		vendor := filepath.Join(artifactDir, "vendor_profile.yaml")
		if err := helpers.WriteP1VendorProfile(vendor, runtimeConfig.RangefinderIMU.X2VirtualSerialLink); err != nil {
			return nil, err
		}
		generated = append(generated, GeneratedRuntimeArtifact{Type: "vendor_profile", Path: vendor})
	}
	if hasHelper(plan, "sensors") {
		sensorConfig := filepath.Join(artifactDir, "gazebo_sensor_runtime.toml")
		vendorProfile, err := runtimeContainerPath(project, filepath.Join(artifactDir, "vendor_profile.yaml"))
		if err != nil {
			return nil, err
		}
		if err := helpers.WriteP2SensorConfig(sensorConfig, vendorProfile, p2Spec(runtimeConfig)); err != nil {
			return nil, err
		}
		generated = append(generated, GeneratedRuntimeArtifact{Type: "sensor_runtime_config", Path: sensorConfig})
		if err := writeGeneratedScript(filepath.Join(artifactDir, "rangefinder_probe.py"), func() (string, error) {
			return helpers.P2RangefinderProbeScript(p2Spec(runtimeConfig))
		}); err != nil {
			return nil, err
		}
		generated = append(generated, GeneratedRuntimeArtifact{Type: "rangefinder_probe_script", Path: filepath.Join(artifactDir, "rangefinder_probe.py")})
		if err := writeGeneratedScript(filepath.Join(artifactDir, "imu_probe.py"), func() (string, error) {
			return helpers.P2IMUProbeScript(p2Spec(runtimeConfig))
		}); err != nil {
			return nil, err
		}
		generated = append(generated, GeneratedRuntimeArtifact{Type: "imu_probe_script", Path: filepath.Join(artifactDir, "imu_probe.py")})
	}
	if hasHelper(plan, "slam") {
		path := filepath.Join(artifactDir, "slam_runtime.toml")
		if err := helpers.WriteP3SlamRuntimeConfig(path, slamSpec(runtimeConfig)); err != nil {
			return nil, err
		}
		generated = append(generated, GeneratedRuntimeArtifact{Type: "slam_runtime_config", Path: path})
	}
	if hasHelper(plan, "fcu-controller") {
		path := filepath.Join(artifactDir, "fcu_controller_runtime.toml")
		spec := fcuSpec(runtimeConfig)
		spec.LandingPolicy = landingPolicyForTask(runtimeConfig, plan.TaskID)
		if hasHelper(plan, "exploration-workflow") {
			exploration := explorationSpec(runtimeConfig)
			spec.ExplorationStatusTopic = exploration.ExplorationStatusTopic
			spec.MotionSpeedMPS = exploration.MotionSpeedMPS
			spec.MinAcceptedGoals = exploration.MinAcceptedGoals
			spec.MinPathLengthM = exploration.MinPathLengthM
		}
		if hasHelper(plan, "scan-stabilization") || hasHelper(plan, "scan-robustness-workflow") {
			spec.IMUInputTopic = runtimeConfig.AirframeDisturbance.IMUInputTopic
			spec.IMUTopic = runtimeConfig.AirframeDisturbance.IMUOutputTopic
			spec.ScanInputTopic = runtimeConfig.ScanStabilization.InputScanTopic
			spec.ScanOutputTopic = runtimeConfig.ScanStabilization.OutputScanTopic
			spec.ScanStabilizationTopic = runtimeConfig.ScanStabilization.StatusTopic
			spec.DisturbanceStatusTopic = runtimeConfig.AirframeDisturbance.StatusTopic
			spec.DisturbanceProfile = runtimeConfig.AirframeDisturbance.Profile
			spec.RequiredProfiles = append([]string(nil), runtimeConfig.AirframeDisturbanceGate.RequiredProfiles...)
		}
		if err := helpers.WriteP4RuntimeConfig(path, spec); err != nil {
			return nil, err
		}
		generated = append(generated, GeneratedRuntimeArtifact{Type: "fcu_runtime_config", Path: path})
		if err := writeGeneratedScript(filepath.Join(artifactDir, "fcu_controller_runtime.py"), func() (string, error) {
			return helpers.FCUControllerRuntimeScript(spec, plan.DurationSec)
		}); err != nil {
			return nil, err
		}
		generated = append(generated, GeneratedRuntimeArtifact{Type: "fcu_runtime_script", Path: filepath.Join(artifactDir, "fcu_controller_runtime.py")})
	}
	if hasHelper(plan, "frame-contract") {
		path := filepath.Join(artifactDir, "frame_contract_runtime.toml")
		spec := frameSpec(runtimeConfig)
		if err := helpers.WriteP5RuntimeConfig(path, spec); err != nil {
			return nil, err
		}
		generated = append(generated, GeneratedRuntimeArtifact{Type: "frame_contract_runtime_config", Path: path})
		if err := writeGeneratedScript(filepath.Join(artifactDir, "frame_contract_probe.py"), func() (string, error) {
			return helpers.FrameContractProbeScript(spec)
		}); err != nil {
			return nil, err
		}
		generated = append(generated, GeneratedRuntimeArtifact{Type: "frame_contract_probe_script", Path: filepath.Join(artifactDir, "frame_contract_probe.py")})
	}
	if hasHelper(plan, "slam-hover") {
		path := filepath.Join(artifactDir, "slam_hover_runtime.toml")
		spec := hoverSpec(runtimeConfig)
		if err := helpers.WriteP6RuntimeConfig(path, spec); err != nil {
			return nil, err
		}
		generated = append(generated, GeneratedRuntimeArtifact{Type: "slam_hover_runtime_config", Path: path})
		if err := writeGeneratedScript(filepath.Join(artifactDir, "slam_hover_probe.py"), func() (string, error) {
			return helpers.HoverProbeScript(spec)
		}); err != nil {
			return nil, err
		}
		generated = append(generated, GeneratedRuntimeArtifact{Type: "slam_hover_probe_script", Path: filepath.Join(artifactDir, "slam_hover_probe.py")})
		notesPath := filepath.Join(artifactDir, "motion_foxglove_notes.md")
		if err := os.WriteFile(notesPath, []byte(helpers.MotionFoxgloveNotes(helpers.DefaultMotionGateSpec())), 0o644); err != nil {
			return nil, err
		}
		generated = append(generated, GeneratedRuntimeArtifact{Type: "foxglove_notes", Path: notesPath})
	}
	if hasHelper(plan, "exploration-workflow") {
		path := filepath.Join(artifactDir, "p8_exploration_runtime.toml")
		spec := explorationSpec(runtimeConfig)
		if err := helpers.WriteP8RuntimeConfig(path, spec); err != nil {
			return nil, err
		}
		generated = append(generated, GeneratedRuntimeArtifact{Type: "p8_runtime_config", Path: path})
		if err := writeGeneratedScript(filepath.Join(artifactDir, "exploration_probe.py"), func() (string, error) {
			return helpers.ExplorationProbeScript(spec)
		}); err != nil {
			return nil, err
		}
		generated = append(generated, GeneratedRuntimeArtifact{Type: "exploration_probe_script", Path: filepath.Join(artifactDir, "exploration_probe.py")})
	}
	if hasHelper(plan, "scan-stabilization") {
		path := filepath.Join(artifactDir, "p11_scan_stabilization_runtime.toml")
		spec := scanStabilizationSpec(runtimeConfig)
		if err := helpers.WriteP11RuntimeConfig(path, spec); err != nil {
			return nil, err
		}
		generated = append(generated, GeneratedRuntimeArtifact{Type: "p11_runtime_config", Path: path})
		if err := writeGeneratedScript(filepath.Join(artifactDir, "stabilization_status_probe.py"), func() (string, error) {
			return helpers.StabilizationStatusProbeScript(spec)
		}); err != nil {
			return nil, err
		}
		generated = append(generated, GeneratedRuntimeArtifact{Type: "stabilization_status_probe_script", Path: filepath.Join(artifactDir, "stabilization_status_probe.py")})
	}
	if hasHelper(plan, "scan-robustness-workflow") {
		path := filepath.Join(artifactDir, "p12_airframe_disturbance_runtime.toml")
		spec := scanRobustnessSpec(runtimeConfig)
		if err := helpers.WriteP12RuntimeConfig(path, spec); err != nil {
			return nil, err
		}
		generated = append(generated, GeneratedRuntimeArtifact{Type: "p12_runtime_config", Path: path})
		bridge := filepath.Join(artifactDir, "p12_bridge_override.yaml")
		if err := helpers.WriteP12BridgeOverride(bridge, runtimeConfig.AirframeDisturbance.IMUInputTopic); err != nil {
			return nil, err
		}
		generated = append(generated, GeneratedRuntimeArtifact{Type: "p12_bridge_override", Path: bridge})
		if err := writeGeneratedScript(filepath.Join(artifactDir, "airframe_disturbance_probe.py"), func() (string, error) {
			return helpers.AirframeDisturbanceProbeScript(spec)
		}); err != nil {
			return nil, err
		}
		generated = append(generated, GeneratedRuntimeArtifact{Type: "airframe_disturbance_probe_script", Path: filepath.Join(artifactDir, "airframe_disturbance_probe.py")})
	}
	return generated, nil
}

func runtimeContainerPath(project config.ProjectConfig, hostPath string) (string, error) {
	workspaceRoot := project.Paths.WorkspaceRoot
	if workspaceRoot == "" {
		workspaceRoot = "."
	}
	absoluteWorkspaceRoot, err := filepath.Abs(workspaceRoot)
	if err != nil {
		return "", err
	}
	containerWorkspace := project.Orchestration.Runtime.Docker.WorkspaceContainerPath
	if containerWorkspace == "" {
		containerWorkspace = "/workspace"
	}
	return containerPath(absoluteWorkspaceRoot, containerWorkspace, hostPath)
}

func writeGeneratedScript(path string, generate func() (string, error)) error {
	content, err := generate()
	if err != nil {
		return err
	}
	return os.WriteFile(path, []byte(content), 0o755)
}

func hasHelper(plan Plan, id string) bool {
	for _, helper := range plan.Helpers {
		if helper.ID == id {
			return true
		}
	}
	return false
}

func p2Spec(runtimeConfig config.TaskRuntimeConfig) helpers.P2SensorSpec {
	spec := helpers.DefaultP2SensorSpec()
	p2 := runtimeConfig.RangefinderIMU
	spec.X2VirtualSerialLink = p2.X2VirtualSerialLink
	spec.X2ScanInputTopic = p2.X2ScanInputTopic
	spec.X2ScanTopic = p2.X2ScanTopic
	spec.X2StatusTopic = p2.X2StatusTopic
	spec.RangefinderFrameID = p2.RangefinderFrameID
	spec.RangefinderScanIdealTopic = p2.RangefinderScanIdealTopic
	spec.RangefinderRangeTopic = p2.RangefinderRangeTopic
	spec.RangefinderStatusTopic = p2.RangefinderStatusTopic
	spec.RangefinderEndpoint = p2.RangefinderEndpoint
	spec.RangefinderMAVOrientation = p2.RangefinderMAVLinkOrientation
	spec.RangefinderRateHz = p2.RangefinderRateHz
	spec.RangefinderMinDistanceM = p2.RangefinderMinDistanceM
	spec.RangefinderMaxDistanceM = p2.RangefinderMaxDistanceM
	spec.RangefinderModelPose = p2.RangefinderModelPose
	spec.RangefinderModelUpdateHz = p2.RangefinderModelUpdateRateHz
	if count, err := strconv.Atoi(p2.RangefinderModelRayCount); err == nil {
		spec.RangefinderModelRayCount = count
	}
	spec.IMUOutputTopic = p2.IMUOutputTopic
	spec.IMUSourceRoute = p2.IMUSourceRoute
	spec.IMUSourceTopic = p2.IMUSourceTopic
	spec.SyntheticFallbackEnabled = p2.SyntheticFallbackEnabled
	return spec
}

func slamSpec(runtimeConfig config.TaskRuntimeConfig) helpers.SlamRuntimeSpec {
	spec := helpers.DefaultSlamRuntimeSpec()
	slam := runtimeConfig.SlamBackend
	spec.Backend = slam.Backend
	spec.LaunchPackage = slam.LaunchPackage
	spec.LaunchFile = slam.LaunchFile
	spec.CartographerConfigurationBasename = slam.CartographerConfigurationBasename
	spec.ScanTopic = slam.ScanTopic
	spec.IMUTopic = slam.IMUTopic
	spec.OdometryTopic = slam.OdometryTopic
	spec.SlamOdomTopic = slam.SlamOdomTopic
	spec.SlamStatusTopic = slam.SlamStatusTopic
	spec.ExternalNavStatusTopic = slam.ExternalNavStatusTopic
	spec.OdomFrameID = slam.OdomFrameID
	spec.BaseFrameID = slam.BaseFrameID
	spec.IMUFrameID = slam.IMUFrameID
	spec.LaserFrameID = slam.LaserFrameID
	return spec
}

func fcuSpec(runtimeConfig config.TaskRuntimeConfig) helpers.FCUControllerSpec {
	spec := helpers.DefaultFCUControllerSpec()
	fcu := runtimeConfig.FCUController
	spec.ControlRoute = fcu.ControlRoute
	spec.MAVLinkBootstrap = fcu.MAVLinkBootstrapEndpoint
	spec.OwnerName = fcu.OwnerName
	spec.OwnerID = fcu.OwnerID
	spec.FCUStateTopic = fcu.FCUStateTopic
	spec.ControllerStatusTopic = fcu.ControllerStatusTopic
	spec.SetpointIntentTopic = fcu.SetpointIntentTopic
	spec.SetpointOutputTopic = fcu.SetpointOutputTopic
	spec.OwnerStatusTopic = fcu.OwnerStatusTopic
	spec.TimeTopic = fcu.TimeTopic
	spec.PrearmService = fcu.PrearmService
	spec.ModeSwitchService = fcu.ModeSwitchService
	spec.ArmService = fcu.ArmService
	spec.TakeoffService = fcu.TakeoffService
	spec.CmdVelTopic = fcu.CmdVelTopic
	spec.PoseTopic = fcu.PoseTopic
	spec.TwistTopic = fcu.TwistTopic
	spec.StatusTopic = fcu.StatusTopic
	spec.RangefinderRangeTopic = fcu.RangefinderRangeTopic
	spec.RangefinderStatusTopic = fcu.RangefinderStatusTopic
	spec.IMUTopic = fcu.IMUTopic
	spec.SlamOdomTopic = fcu.SlamOdomTopic
	spec.SlamStatusTopic = fcu.SlamStatusTopic
	spec.TakeoffAltM = fcu.TakeoffAltM
	spec.ReadinessTimeoutSec = fcu.ReadinessTimeoutSec
	spec.HoldAfterReadySec = fcu.HoldAfterReadySec
	spec.RequireSlamBackend = fcu.RequireSlamBackend
	return spec
}

func landingPolicyForTask(runtimeConfig config.TaskRuntimeConfig, taskID string) string {
	policy := ""
	switch taskID {
	case "hover":
		policy = runtimeConfig.Landing.HoverPolicy
	case "exploration":
		policy = runtimeConfig.Landing.ExplorationPolicy
	case "scan-robustness":
		policy = runtimeConfig.Landing.ScanRobustnessPolicy
	}
	if policy == "" {
		policy = runtimeConfig.Landing.DefaultPolicy
	}
	if policy == "" {
		policy = helpers.PolicyLandInPlace
	}
	return policy
}

func frameSpec(runtimeConfig config.TaskRuntimeConfig) helpers.FrameContractSpec {
	spec := helpers.DefaultFrameContractSpec()
	frame := runtimeConfig.FrameContract
	spec.RequiredFrames = frame.RequiredFrames
	spec.MapFrameID = frame.MapFrameID
	spec.OdomFrameID = frame.OdomFrameID
	spec.BaseFrameID = frame.BaseFrameID
	spec.IMUFrameID = frame.IMUFrameID
	spec.LaserFrameID = frame.LaserFrameID
	spec.RangefinderFrameID = frame.RangefinderFrameID
	spec.ScanTopic = frame.ScanTopic
	spec.IMUTopic = frame.IMUTopic
	spec.RangefinderRangeTopic = frame.RangefinderRangeTopic
	spec.RangefinderStatusTopic = frame.RangefinderStatusTopic
	spec.FCUPoseTopic = frame.FCUPoseTopic
	spec.FCUTwistTopic = frame.FCUTwistTopic
	spec.FCUStatusTopic = frame.FCUStatusTopic
	spec.CmdVelTopic = frame.CmdVelTopic
	spec.SlamOdomTopic = frame.SlamOdomTopic
	spec.SlamStatusTopic = frame.SlamStatusTopic
	spec.TruthDiagnosticTopic = frame.TruthDiagnosticTopic
	spec.ControllerStatusTopic = frame.ControllerStatusTopic
	spec.SetpointOutputTopic = frame.SetpointOutputTopic
	spec.OwnerStatusTopic = frame.OwnerStatusTopic
	spec.StatusTopic = frame.StatusTopic
	spec.MaxDynamicTFAgeSec = frame.MaxDynamicTFAgeSec
	spec.MinScanValidRatio = frame.MinScanValidRatio
	spec.MaxRangefinderHeightErr = frame.MaxRangefinderHeightErrorM
	spec.MaxDirectionErrorRad = frame.MaxDirectionErrorRad
	spec.ProbeDurationSec = frame.ProbeDurationSec
	return spec
}

func hoverSpec(runtimeConfig config.TaskRuntimeConfig) helpers.SlamHoverSpec {
	spec := helpers.DefaultSlamHoverSpec()
	hover := runtimeConfig.SlamHover
	spec.SlamOdomTopic = hover.SlamOdomTopic
	spec.SlamStatusTopic = hover.SlamStatusTopic
	spec.ExternalNavStatusTopic = hover.ExternalNavStatusTopic
	spec.FCUPoseTopic = hover.FCUPoseTopic
	spec.FCUTwistTopic = hover.FCUTwistTopic
	spec.FCUStatusTopic = hover.FCUStatusTopic
	spec.CmdVelTopic = hover.CmdVelTopic
	spec.RangefinderRangeTopic = hover.RangefinderRangeTopic
	spec.RangefinderStatusTopic = hover.RangefinderStatusTopic
	spec.IMUTopic = hover.IMUTopic
	spec.TruthDiagnosticTopic = hover.TruthDiagnosticTopic
	spec.ControllerStatusTopic = hover.ControllerStatusTopic
	spec.SetpointIntentTopic = hover.SetpointIntentTopic
	spec.SetpointOutputTopic = hover.SetpointOutputTopic
	spec.OwnerStatusTopic = hover.OwnerStatusTopic
	spec.HoverStatusTopic = hover.HoverStatusTopic
	spec.VehicleMarkerTopic = hover.VehicleMarkerTopic
	spec.VehicleMarkerPoseTopic = hover.VehicleMarkerPoseTopic
	spec.VehicleMarkerFrameID = hover.VehicleMarkerFrameID
	spec.VehicleMarkerRateHz = hover.VehicleMarkerRateHz
	spec.SettleWindowSec = hover.SettleWindowSec
	spec.HoverWindowSec = hover.HoverWindowSec
	spec.FinalHoldWindowSec = hover.FinalHoldWindowSec
	spec.MaxHoverHorizontalDrift = hover.MaxHoverHorizontalDriftM
	spec.MaxHoverAltitudeError = hover.MaxHoverAltitudeErrorM
	spec.MaxHoverYawDriftRad = hover.MaxHoverYawDriftRad
	spec.MaxStopDriftM = hover.MaxStopDriftM
	return spec
}

func explorationSpec(runtimeConfig config.TaskRuntimeConfig) helpers.ExplorationWorkflowSpec {
	spec := helpers.DefaultExplorationWorkflowSpec()
	exploration := runtimeConfig.ExplorationGate
	spec.Strategy = exploration.Strategy
	spec.ExplorationWindowSec = exploration.ExplorationWindowSec
	spec.MotionSpeedMPS = exploration.MotionSpeedMPS
	spec.MinAcceptedGoals = exploration.MinAcceptedGoals
	spec.MinPathLengthM = exploration.MinPathLengthM
	spec.ControllerStatusTopic = exploration.ControllerStatusTopic
	spec.SetpointIntentTopic = exploration.SetpointIntentTopic
	spec.SetpointOutputTopic = exploration.SetpointOutputTopic
	spec.SlamOdomTopic = exploration.SlamOdomTopic
	spec.ExplorationStatusTopic = exploration.ExplorationStatusTopic
	return spec
}

func scanStabilizationSpec(runtimeConfig config.TaskRuntimeConfig) helpers.ScanStabilizationSpec {
	spec := helpers.DefaultScanStabilizationSpec()
	p11 := runtimeConfig.ScanStabilization
	spec.Mode = p11.Mode
	spec.InputScanTopic = p11.InputScanTopic
	spec.OutputScanTopic = p11.OutputScanTopic
	spec.StatusTopic = p11.StatusTopic
	spec.AttitudeSourceTopic = p11.AttitudeSourceTopic
	spec.MaxRejectedBeamRatio = p11.MaxRejectedBeamRatio
	spec.MinRetainedBeamRatio = p11.MinRetainedBeamRatio
	spec.MaxFloorHitRiskBeamRatio = p11.MaxFloorHitRiskBeamRatio
	spec.MaxVerticalProjectionErrorM = p11.MaxVerticalProjectionErrorM
	spec.MaxAttitudeSourceAgeMS = p11.MaxAttitudeSourceAgeMS
	spec.PassthroughTiltDeg = p11.PassthroughTiltDeg
	spec.CompensationTiltDeg = p11.CompensationTiltDeg
	spec.HardDropTiltDeg = p11.HardDropTiltDeg
	spec.UsesGazeboTruthAsInput = p11.UsesGazeboTruthAsInput
	spec.ScanStabilizationClaim = p11.ScanStabilizationClaim
	spec.ReplayReadinessTimeoutSec = runtimeConfig.ScanStabilizationGate.ReplayReadinessTimeoutSec
	spec.ControllerSummaryTimeoutSec = runtimeConfig.ScanStabilizationGate.ControllerSummaryTimeoutSec
	return spec
}

func scanRobustnessSpec(runtimeConfig config.TaskRuntimeConfig) helpers.ScanRobustnessWorkflowSpec {
	spec := helpers.DefaultScanRobustnessWorkflowSpec()
	disturbance := runtimeConfig.AirframeDisturbance
	gate := runtimeConfig.AirframeDisturbanceGate
	spec.Profile = disturbance.Profile
	spec.RequiredProfiles = gate.RequiredProfiles
	spec.FCUStatusTopic = runtimeConfig.FCUController.ControllerStatusTopic
	if spec.FCUStatusTopic == "" {
		spec.FCUStatusTopic = gate.FCUStatusTopic
	}
	spec.StabilizedScanTopic = runtimeConfig.ScanStabilization.OutputScanTopic
	spec.DisturbanceStatusTopic = disturbance.StatusTopic
	spec.IMURawTopic = disturbance.IMUInputTopic
	spec.IMUOutputTopic = disturbance.IMUOutputTopic
	spec.LandingPolicy = runtimeConfig.Landing.ScanRobustnessPolicy
	return spec
}
