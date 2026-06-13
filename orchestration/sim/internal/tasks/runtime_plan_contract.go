package tasks

import (
	"strings"

	simruntime "navlab/orchestration-sim/internal/runtime"
)

const runtimeBackendDocker = "RUNTIME_BACKEND_DOCKER"

func BuildRuntimePlanContract(plan Plan, runID string, bundle RuntimeSpecBundle) (map[string]any, error) {
	rosbags := make([]map[string]any, 0, len(bundle.Rosbags))
	for _, spec := range bundle.Rosbags {
		topics, err := spec.Topics()
		if err != nil {
			return nil, err
		}
		rosbags = append(rosbags, rosbagContract(spec, topics))
	}
	return map[string]any{
		"schemaVersion": "navlab.runtime.runtime_plan.v1",
		"taskId":        plan.TaskID,
		"runId":         runID,
		"services":      serviceContracts(bundle.Services),
		"probes":        probeContracts(bundle.Probes),
		"rosbags":       rosbags,
	}, nil
}

func serviceContracts(specs []simruntime.ServiceSpec) []map[string]any {
	contracts := make([]map[string]any, 0, len(specs))
	for _, spec := range specs {
		contracts = append(contracts, map[string]any{
			"name":     spec.Name,
			"role":     spec.ServiceRole,
			"backend":  runtimeBackendDocker,
			"image":    spec.Image,
			"command":  append([]string(nil), spec.Command...),
			"env":      copyStringMap(spec.Env),
			"cwd":      spec.CWD,
			"volumes":  volumeContracts(spec.Volumes),
			"networks": append([]string(nil), spec.Networks...),
			"required": spec.Required,
			"logPath":  spec.LogPath,
		})
	}
	return contracts
}

func probeContracts(specs []simruntime.ProbeSpec) []map[string]any {
	contracts := make([]map[string]any, 0, len(specs))
	for _, spec := range specs {
		contracts = append(contracts, map[string]any{
			"name":       spec.Name,
			"role":       spec.ServiceRole,
			"backend":    runtimeBackendDocker,
			"image":      spec.Image,
			"command":    append([]string(nil), spec.Command...),
			"env":        copyStringMap(spec.Env),
			"outputPath": spec.OutputPath,
			"timeoutSec": spec.TimeoutSec,
			"required":   spec.Required,
			"logPath":    spec.LogPath,
		})
	}
	return contracts
}

func rosbagContract(spec simruntime.RosbagSpec, topics []string) map[string]any {
	return map[string]any{
		"name":        spec.Name,
		"role":        spec.ServiceRole,
		"backend":     runtimeBackendDocker,
		"topics":      append([]string(nil), topics...),
		"outputPath":  spec.OutputPath,
		"durationSec": spec.DurationSec,
		"storage":     spec.Storage,
		"required":    spec.Required,
		"logPath":     spec.LogPath,
	}
}

func volumeContracts(mounts []simruntime.VolumeMount) []map[string]any {
	contracts := make([]map[string]any, 0, len(mounts))
	for _, mount := range mounts {
		contracts = append(contracts, map[string]any{
			"source":   mount.Source,
			"target":   mount.Target,
			"readOnly": volumeReadOnly(mount.Mode),
		})
	}
	return contracts
}

func volumeReadOnly(mode string) bool {
	for _, part := range strings.Split(mode, ",") {
		if strings.TrimSpace(part) == "ro" {
			return true
		}
	}
	return false
}

func copyStringMap(values map[string]string) map[string]string {
	copied := make(map[string]string, len(values))
	for key, value := range values {
		copied[key] = value
	}
	return copied
}
