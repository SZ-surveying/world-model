package tasks

import (
	"errors"
	"strings"
	"testing"
	"time"

	simruntime "navlab/orchestration-sim/internal/runtime"
)

type fakeRuntimeBackend struct {
	events           []string
	failService      string
	probeResults     map[string]simruntime.ProbeResult
	probeErrors      map[string]error
	waitReturnCodes  map[string]int
	waitErrors       map[string]error
	stopErrors       map[string]error
	serviceStartTime time.Time
}

type captureEventSink struct {
	events []RuntimeEvent
}

func (sink *captureEventSink) EmitRuntimeEvent(event RuntimeEvent) {
	sink.events = append(sink.events, event)
}

func (backend *fakeRuntimeBackend) StartService(spec simruntime.ServiceSpec) (simruntime.RuntimeHandle, error) {
	backend.events = append(backend.events, "start-service:"+spec.Name)
	if spec.Name == backend.failService {
		return simruntime.RuntimeHandle{}, errors.New("boom")
	}
	return simruntime.RuntimeHandle{
		Backend:     "fake",
		ServiceName: spec.Name,
		Identifier:  spec.Name,
		StartedAt:   backend.serviceStartTime,
	}, nil
}

func (backend *fakeRuntimeBackend) StartRosbag(spec simruntime.RosbagSpec) (simruntime.RuntimeHandle, error) {
	backend.events = append(backend.events, "start-rosbag:"+spec.Name)
	return simruntime.RuntimeHandle{
		Backend:     "fake",
		ServiceName: spec.Name,
		Identifier:  spec.Name,
		StartedAt:   backend.serviceStartTime,
	}, nil
}

func (backend *fakeRuntimeBackend) RunProbe(spec simruntime.ProbeSpec) (simruntime.ProbeResult, error) {
	backend.events = append(backend.events, "probe:"+spec.Name)
	if result, ok := backend.probeResults[spec.Name]; ok {
		return result, backend.probeErrors[spec.Name]
	}
	return simruntime.ProbeResult{Backend: "fake", Name: spec.Name, ReturnCode: 0}, backend.probeErrors[spec.Name]
}

func (backend *fakeRuntimeBackend) Wait(handle simruntime.RuntimeHandle) (int, error) {
	backend.events = append(backend.events, "wait:"+handle.ServiceName)
	return backend.waitReturnCodes[handle.ServiceName], backend.waitErrors[handle.ServiceName]
}

func (backend *fakeRuntimeBackend) Stop(handle simruntime.RuntimeHandle) error {
	backend.events = append(backend.events, "stop:"+handle.ServiceName)
	return backend.stopErrors[handle.ServiceName]
}

func (backend *fakeRuntimeBackend) Logs(handle simruntime.RuntimeHandle, tail int) (string, error) {
	backend.events = append(backend.events, "logs:"+handle.ServiceName)
	return "", nil
}

func TestExecuteRuntimeSpecsStartsRunsWaitsAndCleansUp(t *testing.T) {
	backend := newFakeRuntimeBackend()
	result, err := ExecuteRuntimeSpecs(backend, RuntimeSpecBundle{
		Services: []simruntime.ServiceSpec{
			{Name: "gazebo"},
			{Name: "slam"},
		},
		Rosbags: []simruntime.RosbagSpec{
			{Name: "hover_rosbag"},
		},
		Probes: []simruntime.ProbeSpec{
			{Name: "frame_probe", Required: true},
		},
	}, RuntimeExecutionOptions{WaitForRosbags: true})
	if err != nil {
		t.Fatalf("ExecuteRuntimeSpecs() error = %v", err)
	}
	if len(result.ServiceHandles) != 2 || len(result.RosbagHandles) != 1 || len(result.ProbeResults) != 1 {
		t.Fatalf("result = %#v", result)
	}
	assertEvents(t, backend.events, []string{
		"start-service:gazebo",
		"start-service:slam",
		"start-rosbag:hover_rosbag",
		"probe:frame_probe",
		"wait:hover_rosbag",
		"stop:hover_rosbag",
		"stop:slam",
		"stop:gazebo",
	})
}

func TestExecuteRuntimeSpecsEmitsRuntimeEvents(t *testing.T) {
	backend := newFakeRuntimeBackend()
	sink := &captureEventSink{}
	_, err := ExecuteRuntimeSpecs(backend, RuntimeSpecBundle{
		Services: []simruntime.ServiceSpec{{Name: "gazebo", LogPath: "gazebo.log"}},
		Rosbags:  []simruntime.RosbagSpec{{Name: "hover_rosbag", LogPath: "rosbag.log", OutputPath: "hover.mcap"}},
		Probes:   []simruntime.ProbeSpec{{Name: "frame_probe", Required: true, LogPath: "probe.log"}},
	}, RuntimeExecutionOptions{WaitForRosbags: true, TaskID: "hover", RunID: "run", EventSink: sink})
	if err != nil {
		t.Fatalf("ExecuteRuntimeSpecs() error = %v", err)
	}
	phases := make([]string, 0, len(sink.events))
	for _, event := range sink.events {
		phases = append(phases, event.Phase)
		if event.TaskID != "hover" || event.RunID != "run" {
			t.Fatalf("event task/run = %q/%q", event.TaskID, event.RunID)
		}
	}
	for _, want := range []string{
		"run.started",
		"service.starting",
		"service.started",
		"rosbag.starting",
		"rosbag.started",
		"probe.running",
		"probe.finished",
		"rosbag.waiting",
		"rosbag.finished",
		"rosbag.stopping",
		"service.stopping",
		"run.completed",
	} {
		if !containsString(phases, want) {
			t.Fatalf("phases missing %q: %#v", want, phases)
		}
	}
}

func TestExecuteRuntimeSpecsFailsRequiredProbeAndCleansUp(t *testing.T) {
	backend := newFakeRuntimeBackend()
	backend.probeResults["frame_probe"] = simruntime.ProbeResult{Backend: "fake", Name: "frame_probe", ReturnCode: 42}

	_, err := ExecuteRuntimeSpecs(backend, RuntimeSpecBundle{
		Services: []simruntime.ServiceSpec{{Name: "gazebo"}},
		Probes:   []simruntime.ProbeSpec{{Name: "frame_probe", Required: true}},
	}, RuntimeExecutionOptions{})
	if err == nil || !strings.Contains(err.Error(), "required probes failed") {
		t.Fatalf("ExecuteRuntimeSpecs() error = %v, want required probe failure", err)
	}
	assertEvents(t, backend.events, []string{
		"start-service:gazebo",
		"probe:frame_probe",
		"stop:gazebo",
	})
}

func TestExecuteRuntimeSpecsCleansUpAfterServiceStartFailure(t *testing.T) {
	backend := newFakeRuntimeBackend()
	backend.failService = "slam"

	_, err := ExecuteRuntimeSpecs(backend, RuntimeSpecBundle{
		Services: []simruntime.ServiceSpec{
			{Name: "gazebo"},
			{Name: "slam"},
		},
	}, RuntimeExecutionOptions{})
	if err == nil || !strings.Contains(err.Error(), "start service slam") {
		t.Fatalf("ExecuteRuntimeSpecs() error = %v, want start service failure", err)
	}
	assertEvents(t, backend.events, []string{
		"start-service:gazebo",
		"start-service:slam",
		"stop:gazebo",
	})
}

func newFakeRuntimeBackend() *fakeRuntimeBackend {
	return &fakeRuntimeBackend{
		probeResults:     map[string]simruntime.ProbeResult{},
		probeErrors:      map[string]error{},
		waitReturnCodes:  map[string]int{},
		waitErrors:       map[string]error{},
		stopErrors:       map[string]error{},
		serviceStartTime: time.Date(2026, 6, 12, 1, 2, 3, 0, time.UTC),
	}
}

func assertEvents(t *testing.T, got []string, want []string) {
	t.Helper()
	if strings.Join(got, "\n") != strings.Join(want, "\n") {
		t.Fatalf("events = %#v, want %#v", got, want)
	}
}

func containsString(values []string, target string) bool {
	for _, value := range values {
		if value == target {
			return true
		}
	}
	return false
}
