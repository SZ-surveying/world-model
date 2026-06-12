package tasks

import (
	"fmt"
	"os"
	"path/filepath"
	"strings"

	"navlab/orchestration-sim/internal/config"
	simruntime "navlab/orchestration-sim/internal/runtime"
	"navlab/orchestration-sim/internal/tasks/helpers"
)

type RuntimeSpecBundle struct {
	Services []simruntime.ServiceSpec `json:"services"`
	Probes   []simruntime.ProbeSpec   `json:"probes"`
	Rosbags  []simruntime.RosbagSpec  `json:"rosbags"`
}

func BuildRuntimeSpecs(project config.ProjectConfig, plan helpers.ExecutionPlan, artifactDir string) (RuntimeSpecBundle, error) {
	var bundle RuntimeSpecBundle
	workspaceRoot := project.Paths.WorkspaceRoot
	if workspaceRoot == "" {
		workspaceRoot = "."
	}
	absoluteWorkspaceRoot, err := filepath.Abs(workspaceRoot)
	if err != nil {
		return RuntimeSpecBundle{}, err
	}
	containerWorkspace := project.Orchestration.Runtime.Docker.WorkspaceContainerPath
	if containerWorkspace == "" {
		containerWorkspace = "/workspace"
	}
	workspaceMount := simruntime.VolumeMount{
		Source: absoluteWorkspaceRoot,
		Target: containerWorkspace,
	}
	containerArtifactDir, err := containerPath(absoluteWorkspaceRoot, containerWorkspace, artifactDir)
	if err != nil {
		return RuntimeSpecBundle{}, err
	}

	if usesOfficialBaseline(plan) {
		spec, err := officialBaselineServiceSpec(project, workspaceMount, containerWorkspace, containerArtifactDir, artifactDir)
		if err != nil {
			return RuntimeSpecBundle{}, err
		}
		if err := spec.ValidateDocker(); err != nil {
			return RuntimeSpecBundle{}, err
		}
		bundle.Services = append(bundle.Services, spec)
	}

	for _, service := range plan.RuntimeServices {
		image, err := resolveImageRef(project, service.ImageRef)
		if err != nil {
			return RuntimeSpecBundle{}, err
		}
		command := rewriteArtifactReferences(service.Command, containerArtifactDir)
		env := rewriteArtifactEnv(resolveEnv(project, service.Env), containerArtifactDir)
		spec := simruntime.ServiceSpec{
			Name:          service.ServiceName,
			Image:         image,
			ContainerName: service.ContainerName,
			Command:       command,
			Env:           env,
			CWD:           containerWorkspace,
			Volumes:       []simruntime.VolumeMount{workspaceMount},
			Networks:      networks(service.Network),
			Detach:        true,
			Required:      true,
			LogPath:       filepath.Join(artifactDir, service.ServiceName+".start.log"),
			ServiceRole:   service.HelperID,
		}
		if err := spec.ValidateDocker(); err != nil {
			return RuntimeSpecBundle{}, err
		}
		bundle.Services = append(bundle.Services, spec)
	}

	for _, probe := range plan.ROSProbes {
		image, err := resolveImageRef(project, probe.RuntimeImage)
		if err != nil {
			return RuntimeSpecBundle{}, err
		}
		scriptPath := filepath.Join(artifactDir, probe.ScriptPath)
		outputPath := filepath.Join(artifactDir, probe.OutputPath)
		containerScriptPath, err := containerPath(absoluteWorkspaceRoot, containerWorkspace, scriptPath)
		if err != nil {
			return RuntimeSpecBundle{}, err
		}
		containerOutputPath, err := containerPath(absoluteWorkspaceRoot, containerWorkspace, outputPath)
		if err != nil {
			return RuntimeSpecBundle{}, err
		}
		spec := simruntime.ProbeSpec{
			Name:    probe.Name,
			Image:   image,
			Command: []string{"bash", "-lc", "python3 " + shellQuote(containerScriptPath) + " > " + shellQuote(containerOutputPath)},
			Env:     baselineEnv(project),
			CWD:     containerWorkspace,
			Volumes: []simruntime.VolumeMount{workspaceMount},
			Networks: []string{
				"host",
			},
			TimeoutSec:  30,
			LogPath:     filepath.Join(artifactDir, probe.Name+".log"),
			Required:    true,
			ServiceRole: probe.HelperID,
		}
		if err := spec.ValidateDocker(); err != nil {
			return RuntimeSpecBundle{}, err
		}
		bundle.Probes = append(bundle.Probes, spec)
	}

	for _, rosbag := range plan.RosbagRecords {
		image, err := resolveImageRef(project, "images.runtime")
		if err != nil {
			return RuntimeSpecBundle{}, err
		}
		profilePath, err := writeRosbagTopicsProfile(artifactDir, rosbag.Name, rosbag.Topics)
		if err != nil {
			return RuntimeSpecBundle{}, err
		}
		outputPath, err := containerPath(absoluteWorkspaceRoot, containerWorkspace, filepath.Join(artifactDir, rosbag.OutputDir))
		if err != nil {
			return RuntimeSpecBundle{}, err
		}
		spec := simruntime.RosbagSpec{
			Name:          rosbag.Name,
			Image:         image,
			ContainerName: rosbag.Name,
			TopicsProfile: profilePath,
			OutputPath:    outputPath,
			DurationSec:   plan.DurationSec,
			Storage:       "mcap",
			Env:           baselineEnv(project),
			CWD:           containerWorkspace,
			Volumes:       []simruntime.VolumeMount{workspaceMount},
			Networks:      []string{"host"},
			LogPath:       filepath.Join(artifactDir, rosbag.Name+".log"),
			Required:      true,
			ServiceRole:   rosbag.HelperID,
		}
		if _, err := simruntime.RosbagServiceSpec(spec); err != nil {
			return RuntimeSpecBundle{}, err
		}
		bundle.Rosbags = append(bundle.Rosbags, spec)
	}
	return bundle, nil
}

func resolveImageRef(project config.ProjectConfig, ref string) (string, error) {
	key := strings.TrimPrefix(strings.TrimSpace(ref), "images.")
	if key == "" {
		return "", fmt.Errorf("image ref is required")
	}
	if key == "runtime" {
		key = "official_baseline"
	}
	image, ok := project.Images[key]
	if !ok || strings.TrimSpace(image.Repository) == "" {
		return "", fmt.Errorf("image %q is not configured", key)
	}
	repository := image.Repository
	if strings.Contains(repository, ":") {
		return repository, nil
	}
	tag := project.Navlab.Images.TagStrategy
	if tag == "" {
		tag = "latest"
	}
	return repository + ":" + tag, nil
}

func resolveEnv(project config.ProjectConfig, values map[string]string) map[string]string {
	resolved := map[string]string{}
	for key, value := range values {
		switch value {
		case "from config.toml":
			switch key {
			case "ROS_DOMAIN_ID", "DDS_DOMAIN_ID":
				resolved[key] = runtimeRosDomain(project)
			case "RMW_IMPLEMENTATION":
				resolved[key] = "rmw_cyclonedds_cpp"
			case "DDS_ENABLE":
				resolved[key] = "1"
			default:
				resolved[key] = value
			}
		default:
			resolved[key] = value
		}
	}
	return resolved
}

func baselineEnv(project config.ProjectConfig) map[string]string {
	return map[string]string{
		"DDS_ENABLE":         "1",
		"DDS_DOMAIN_ID":      runtimeRosDomain(project),
		"ROS_DOMAIN_ID":      runtimeRosDomain(project),
		"RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
	}
}

func runtimeRosDomain(project config.ProjectConfig) string {
	if strings.TrimSpace(project.Official.DDSDomainID) != "" {
		return project.Official.DDSDomainID
	}
	return project.RosDomainID
}

func usesOfficialBaseline(plan helpers.ExecutionPlan) bool {
	switch plan.TaskID {
	case "hover", "exploration", "scan-robustness":
		return true
	default:
		return false
	}
}

func officialBaselineServiceSpec(
	project config.ProjectConfig,
	workspaceMount simruntime.VolumeMount,
	containerWorkspace string,
	containerArtifactDir string,
	artifactDir string,
) (simruntime.ServiceSpec, error) {
	image, err := resolveImageRef(project, "images.official_baseline")
	if err != nil {
		return simruntime.ServiceSpec{}, err
	}
	launch := strings.TrimSpace(project.Official.GazeboLaunch)
	if launch == "" {
		launch = "ros2 launch ardupilot_gz_bringup iris_maze.launch.py"
	}
	command := strings.Join([]string{
		"source /opt/ros/jazzy/setup.bash",
		"source /opt/navlab_official_ws/install/setup.bash",
		"export NAVLAB_OFFICIAL_SDF_ROOTS=/opt/navlab_official_ws/install/ardupilot_gazebo/share:/opt/navlab_official_ws/install/ardupilot_gz_description/share",
		"export SDF_PATH=${NAVLAB_OFFICIAL_SDF_ROOTS}:${SDF_PATH:-}",
		"export GZ_SIM_RESOURCE_PATH=${NAVLAB_OFFICIAL_SDF_ROOTS}:${GZ_SIM_RESOURCE_PATH:-}",
		"mkdir -p " + shellQuote(containerArtifactDir+"/sitl_work"),
		"cd " + shellQuote(containerArtifactDir+"/sitl_work"),
		"exec " + launch + " use_gz_sim_gui:=false rviz:=false use_dds_agent:=true use_gz_sim_server:=true spawn_robot:=true",
	}, " && ")
	volumes := []simruntime.VolumeMount{workspaceMount}
	p12BridgeOverride := filepath.Join(artifactDir, "p12_bridge_override.yaml")
	if _, err := os.Stat(p12BridgeOverride); err == nil {
		absoluteP12BridgeOverride, err := filepath.Abs(p12BridgeOverride)
		if err != nil {
			return simruntime.ServiceSpec{}, err
		}
		volumes = append(volumes, simruntime.VolumeMount{
			Source: absoluteP12BridgeOverride,
			Target: helpers.OfficialIris3DBridgeConfig,
		})
	} else {
		bridgeOverride := filepath.Join(artifactDir, "bridge_override.yaml")
		if _, err := os.Stat(bridgeOverride); err == nil {
			absoluteBridgeOverride, err := filepath.Abs(bridgeOverride)
			if err != nil {
				return simruntime.ServiceSpec{}, err
			}
			volumes = append(volumes, simruntime.VolumeMount{
				Source: absoluteBridgeOverride,
				Target: helpers.OfficialIris3DBridgeConfig,
			})
		}
	}
	return simruntime.ServiceSpec{
		Name:          "official_baseline",
		Image:         image,
		ContainerName: "navlab-official-baseline",
		Command:       []string{"bash", "-lc", command},
		Env: map[string]string{
			"SESSION_ID":         project.SessionID,
			"ROS_DOMAIN_ID":      runtimeRosDomain(project),
			"DDS_DOMAIN_ID":      runtimeRosDomain(project),
			"DDS_ENABLE":         "1",
			"RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
			"PYTHONPATH":         containerWorkspace,
		},
		CWD:         containerWorkspace,
		Volumes:     volumes,
		Networks:    []string{"host"},
		Detach:      true,
		Required:    true,
		LogPath:     filepath.Join(artifactDir, "official_baseline.start.log"),
		ServiceRole: "official-baseline",
	}, nil
}

func networks(network string) []string {
	if strings.TrimSpace(network) == "" {
		return nil
	}
	return []string{network}
}

func rewriteArtifactReferences(values []string, containerArtifactDir string) []string {
	rewritten := make([]string, 0, len(values))
	for _, value := range values {
		rewritten = append(rewritten, strings.ReplaceAll(value, "artifacts/", containerArtifactDir+"/"))
	}
	return rewritten
}

func rewriteArtifactEnv(values map[string]string, containerArtifactDir string) map[string]string {
	if len(values) == 0 {
		return values
	}
	rewritten := make(map[string]string, len(values))
	for key, value := range values {
		rewritten[key] = strings.ReplaceAll(value, "artifacts/", containerArtifactDir+"/")
	}
	return rewritten
}

func writeRosbagTopicsProfile(artifactDir string, name string, topics []string) (string, error) {
	if len(topics) == 0 {
		return "", fmt.Errorf("rosbag %s has no topics", name)
	}
	path := filepath.Join(artifactDir, "profiles", name+".txt")
	if err := os.MkdirAll(filepath.Dir(path), 0o755); err != nil {
		return "", err
	}
	return path, os.WriteFile(path, []byte(strings.Join(topics, "\n")+"\n"), 0o644)
}

func containerPath(workspaceRoot string, containerWorkspace string, hostPath string) (string, error) {
	cleaned := filepath.Clean(hostPath)
	absoluteHostPath, err := filepath.Abs(cleaned)
	if err != nil {
		return "", err
	}
	relativePath, err := filepath.Rel(workspaceRoot, absoluteHostPath)
	if err == nil && relativePath != "." && !strings.HasPrefix(relativePath, "..") && !filepath.IsAbs(relativePath) {
		return filepath.ToSlash(filepath.Join(containerWorkspace, relativePath)), nil
	}
	if err == nil && relativePath == "." {
		return filepath.ToSlash(containerWorkspace), nil
	}
	if strings.HasPrefix(absoluteHostPath, string(filepath.Separator)) {
		absoluteHostPath = strings.TrimPrefix(absoluteHostPath, string(filepath.Separator))
	}
	return filepath.ToSlash(filepath.Join(containerWorkspace, absoluteHostPath)), nil
}

func shellQuote(value string) string {
	if value == "" {
		return "''"
	}
	return "'" + strings.ReplaceAll(value, "'", "'\"'\"'") + "'"
}
