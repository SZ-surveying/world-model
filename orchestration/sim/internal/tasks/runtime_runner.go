package tasks

import (
	"errors"
	"fmt"
	"strings"

	simruntime "navlab/orchestration-sim/internal/runtime"
)

type RuntimeExecutionOptions struct {
	KeepRunning    bool
	WaitForRosbags bool
}

type RuntimeExecutionResult struct {
	ServiceHandles []simruntime.RuntimeHandle `json:"service_handles"`
	RosbagHandles  []simruntime.RuntimeHandle `json:"rosbag_handles"`
	ProbeResults   []simruntime.ProbeResult   `json:"probe_results"`
	StopErrors     []string                   `json:"stop_errors,omitempty"`
}

func ExecuteRuntimeSpecs(
	backend simruntime.Backend,
	bundle RuntimeSpecBundle,
	options RuntimeExecutionOptions,
) (RuntimeExecutionResult, error) {
	if backend == nil {
		return RuntimeExecutionResult{}, errors.New("runtime backend is required")
	}
	var result RuntimeExecutionResult
	cleanup := func() {
		if options.KeepRunning {
			return
		}
		stopRuntimeHandles(backend, append([]simruntime.RuntimeHandle{}, result.RosbagHandles...), &result)
		stopRuntimeHandles(backend, append([]simruntime.RuntimeHandle{}, result.ServiceHandles...), &result)
	}

	for _, spec := range bundle.Services {
		handle, err := backend.StartService(spec)
		if err != nil {
			cleanup()
			return result, fmt.Errorf("start service %s: %w", spec.Name, err)
		}
		result.ServiceHandles = append(result.ServiceHandles, handle)
	}
	for _, spec := range bundle.Rosbags {
		handle, err := backend.StartRosbag(spec)
		if err != nil {
			cleanup()
			return result, fmt.Errorf("start rosbag %s: %w", spec.Name, err)
		}
		result.RosbagHandles = append(result.RosbagHandles, handle)
	}

	var probeFailures []string
	for _, spec := range bundle.Probes {
		probe, err := backend.RunProbe(spec)
		result.ProbeResults = append(result.ProbeResults, probe)
		if err != nil && spec.Required {
			probeFailures = append(probeFailures, fmt.Sprintf("%s: %v", spec.Name, err))
			continue
		}
		if spec.Required && !probe.OK() {
			probeFailures = append(probeFailures, fmt.Sprintf("%s: return code %d", spec.Name, probe.ReturnCode))
		}
	}
	if len(probeFailures) > 0 {
		cleanup()
		return result, fmt.Errorf("required probes failed: %s", strings.Join(probeFailures, "; "))
	}

	if options.WaitForRosbags {
		for _, handle := range result.RosbagHandles {
			code, err := backend.Wait(handle)
			if err != nil {
				cleanup()
				return result, fmt.Errorf("wait rosbag %s: %w", handle.ServiceName, err)
			}
			if code != 0 && code != 124 {
				cleanup()
				return result, fmt.Errorf("wait rosbag %s: return code %d", handle.ServiceName, code)
			}
		}
	}
	cleanup()
	if len(result.StopErrors) > 0 {
		return result, fmt.Errorf("runtime cleanup failed: %s", strings.Join(result.StopErrors, "; "))
	}
	return result, nil
}

func stopRuntimeHandles(backend simruntime.Backend, handles []simruntime.RuntimeHandle, result *RuntimeExecutionResult) {
	for left, right := 0, len(handles)-1; left < right; left, right = left+1, right-1 {
		handles[left], handles[right] = handles[right], handles[left]
	}
	for _, handle := range handles {
		if err := backend.Stop(handle); err != nil {
			result.StopErrors = append(result.StopErrors, fmt.Sprintf("%s: %v", handle.ServiceName, err))
		}
	}
}
