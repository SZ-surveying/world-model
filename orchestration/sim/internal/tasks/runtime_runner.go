package tasks

import (
	"errors"
	"fmt"
	"strings"
	"time"

	simruntime "navlab/orchestration-sim/internal/runtime"
)

type RuntimeExecutionOptions struct {
	KeepRunning    bool
	WaitForRosbags bool
	TaskID         string
	RunID          string
	EventSink      RuntimeEventSink
}

type RuntimeEventSink interface {
	EmitRuntimeEvent(event RuntimeEvent)
}

type RuntimeEvent struct {
	Time        time.Time      `json:"time"`
	TaskID      string         `json:"task_id,omitempty"`
	RunID       string         `json:"run_id,omitempty"`
	Phase       string         `json:"phase"`
	Component   string         `json:"component,omitempty"`
	ComponentID string         `json:"component_id,omitempty"`
	Level       string         `json:"level,omitempty"`
	Message     string         `json:"message,omitempty"`
	Artifact    string         `json:"artifact,omitempty"`
	Payload     map[string]any `json:"payload,omitempty"`
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
		stopRuntimeHandles(backend, append([]simruntime.RuntimeHandle{}, result.RosbagHandles...), "rosbag", &result, options)
		stopRuntimeHandles(backend, append([]simruntime.RuntimeHandle{}, result.ServiceHandles...), "service", &result, options)
	}

	emitRuntimeEvent(options, RuntimeEvent{Phase: "run.started", Level: "info", Message: "runtime execution started"})
	for _, spec := range bundle.Services {
		emitRuntimeEvent(options, componentEvent("service.starting", "service", spec.Name, spec.LogPath, "starting service"))
		handle, err := backend.StartService(spec)
		if err != nil {
			emitRuntimeEvent(options, componentEvent("service.failed", "service", spec.Name, spec.LogPath, err.Error()))
			cleanup()
			emitRuntimeEvent(options, RuntimeEvent{Phase: "run.failed", Level: "error", Message: err.Error()})
			return result, fmt.Errorf("start service %s: %w", spec.Name, err)
		}
		result.ServiceHandles = append(result.ServiceHandles, handle)
		emitRuntimeEvent(options, componentEvent("service.started", "service", spec.Name, handle.LogPath, "service started"))
	}
	for _, spec := range bundle.Rosbags {
		emitRuntimeEvent(options, componentEvent("rosbag.starting", "rosbag", spec.Name, spec.LogPath, "starting rosbag"))
		handle, err := backend.StartRosbag(spec)
		if err != nil {
			emitRuntimeEvent(options, componentEvent("rosbag.failed", "rosbag", spec.Name, spec.LogPath, err.Error()))
			cleanup()
			emitRuntimeEvent(options, RuntimeEvent{Phase: "run.failed", Level: "error", Message: err.Error()})
			return result, fmt.Errorf("start rosbag %s: %w", spec.Name, err)
		}
		result.RosbagHandles = append(result.RosbagHandles, handle)
		event := componentEvent("rosbag.started", "rosbag", spec.Name, handle.LogPath, "rosbag started")
		event.Artifact = spec.OutputPath
		emitRuntimeEvent(options, event)
	}

	var probeFailures []string
	for _, spec := range bundle.Probes {
		emitRuntimeEvent(options, componentEvent("probe.running", "probe", spec.Name, spec.LogPath, "running probe"))
		probe, err := backend.RunProbe(spec)
		result.ProbeResults = append(result.ProbeResults, probe)
		event := componentEvent("probe.finished", "probe", spec.Name, probe.LogPath, "probe finished")
		event.Payload = map[string]any{"return_code": probe.ReturnCode, "required": spec.Required}
		if err != nil || !probe.OK() {
			event.Phase = "probe.failed"
			event.Level = "warn"
			event.Message = fmt.Sprintf("probe return code %d", probe.ReturnCode)
			if err != nil {
				event.Message = err.Error()
			}
		}
		emitRuntimeEvent(options, event)
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
		err := fmt.Errorf("required probes failed: %s", strings.Join(probeFailures, "; "))
		emitRuntimeEvent(options, RuntimeEvent{Phase: "run.blocked", Level: "error", Message: err.Error()})
		return result, err
	}

	if options.WaitForRosbags {
		for _, handle := range result.RosbagHandles {
			emitRuntimeEvent(options, componentEvent("rosbag.waiting", "rosbag", handle.ServiceName, handle.LogPath, "waiting for rosbag"))
			code, err := backend.Wait(handle)
			if err != nil {
				cleanup()
				emitRuntimeEvent(options, componentEvent("rosbag.failed", "rosbag", handle.ServiceName, handle.LogPath, err.Error()))
				emitRuntimeEvent(options, RuntimeEvent{Phase: "run.failed", Level: "error", Message: err.Error()})
				return result, fmt.Errorf("wait rosbag %s: %w", handle.ServiceName, err)
			}
			if code != 0 && code != 124 {
				cleanup()
				err := fmt.Errorf("wait rosbag %s: return code %d", handle.ServiceName, code)
				emitRuntimeEvent(options, componentEvent("rosbag.failed", "rosbag", handle.ServiceName, handle.LogPath, err.Error()))
				emitRuntimeEvent(options, RuntimeEvent{Phase: "run.failed", Level: "error", Message: err.Error()})
				return result, err
			}
			event := componentEvent("rosbag.finished", "rosbag", handle.ServiceName, handle.LogPath, "rosbag finished")
			event.Payload = map[string]any{"return_code": code}
			emitRuntimeEvent(options, event)
		}
	}
	cleanup()
	if len(result.StopErrors) > 0 {
		err := fmt.Errorf("runtime cleanup failed: %s", strings.Join(result.StopErrors, "; "))
		emitRuntimeEvent(options, RuntimeEvent{Phase: "run.failed", Level: "error", Message: err.Error()})
		return result, err
	}
	emitRuntimeEvent(options, RuntimeEvent{Phase: "run.completed", Level: "info", Message: "runtime execution completed"})
	return result, nil
}

func stopRuntimeHandles(backend simruntime.Backend, handles []simruntime.RuntimeHandle, component string, result *RuntimeExecutionResult, options RuntimeExecutionOptions) {
	for left, right := 0, len(handles)-1; left < right; left, right = left+1, right-1 {
		handles[left], handles[right] = handles[right], handles[left]
	}
	for _, handle := range handles {
		emitRuntimeEvent(options, componentEvent(component+".stopping", component, handle.ServiceName, handle.LogPath, "stopping "+component))
		if err := backend.Stop(handle); err != nil {
			result.StopErrors = append(result.StopErrors, fmt.Sprintf("%s: %v", handle.ServiceName, err))
			emitRuntimeEvent(options, componentEvent(component+".stop_failed", component, handle.ServiceName, handle.LogPath, err.Error()))
			continue
		}
		emitRuntimeEvent(options, componentEvent(component+".stopped", component, handle.ServiceName, handle.LogPath, component+" stopped"))
	}
}

func componentEvent(phase string, component string, componentID string, logPath string, message string) RuntimeEvent {
	return RuntimeEvent{
		Phase:       phase,
		Component:   component,
		ComponentID: componentID,
		Level:       "info",
		Message:     message,
		Artifact:    logPath,
	}
}

func emitRuntimeEvent(options RuntimeExecutionOptions, event RuntimeEvent) {
	if options.EventSink == nil {
		return
	}
	event.Time = time.Now().UTC()
	event.TaskID = options.TaskID
	event.RunID = options.RunID
	options.EventSink.EmitRuntimeEvent(event)
}
