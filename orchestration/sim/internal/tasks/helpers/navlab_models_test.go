package helpers

import (
	"context"
	"os"
	"path/filepath"
	"strings"
	"testing"
)

type fakeRunner struct {
	calls []string
	err   error
}

func (runner *fakeRunner) Run(ctx context.Context, command string, args ...string) (string, error) {
	runner.calls = append(runner.calls, command+" "+strings.Join(args, " "))
	return "ok", runner.err
}

func TestWriteP1BridgeOverrideAndVendorProfile(t *testing.T) {
	dir := t.TempDir()
	bridge := filepath.Join(dir, "bridge.yaml")
	vendor := filepath.Join(dir, "vendor.yaml")
	if err := WriteP1BridgeOverride(bridge); err != nil {
		t.Fatalf("WriteP1BridgeOverride() error = %v", err)
	}
	if err := WriteP1VendorProfile(vendor, "/tmp/x2"); err != nil {
		t.Fatalf("WriteP1VendorProfile() error = %v", err)
	}
	bridgeData, _ := os.ReadFile(bridge)
	vendorData, _ := os.ReadFile(vendor)
	if !strings.Contains(string(bridgeData), "ros_topic_name: \"imu\"") {
		t.Fatalf("bridge override missing imu topic:\n%s", bridgeData)
	}
	if !strings.Contains(string(vendorData), "port: /tmp/x2") {
		t.Fatalf("vendor profile missing serial link:\n%s", vendorData)
	}
}

func TestGazeboSensorDockerArgs(t *testing.T) {
	args, err := GazeboSensorDockerArgs(GazeboSensorRunSpec{
		Image:                  "world-model/navlab-gazebo-sensor",
		SessionID:              "session",
		RosDomainID:            "85",
		RMWImplementation:      "rmw_fastrtps_cpp",
		SensorConfigPath:       "/workspace/artifacts/sensor.toml",
		RuntimeLogPath:         "/workspace/artifacts/gazebo_sensor_runtime.log",
		WorkspaceHostPath:      ".",
		WorkspaceContainerPath: "/workspace",
	})
	if err != nil {
		t.Fatalf("GazeboSensorDockerArgs() error = %v", err)
	}
	joined := strings.Join(args, " ")
	for _, want := range []string{
		"run",
		"--name " + GazeboSensorContainer,
		"NAVLAB_CONFIG=/workspace/artifacts/sensor.toml",
		"world-model/navlab-gazebo-sensor",
		"navlab.sim.gazebo_sensor.cli",
	} {
		if !strings.Contains(joined, want) {
			t.Fatalf("docker args missing %q:\n%s", want, joined)
		}
	}
}

func TestStartGazeboSensorContainerUsesDockerRunner(t *testing.T) {
	runner := &fakeRunner{}
	err := StartGazeboSensorContainer(context.Background(), runner, GazeboSensorRunSpec{
		Image:            "image",
		SensorConfigPath: "/workspace/config.toml",
		RuntimeLogPath:   "/workspace/log.txt",
	})
	if err != nil {
		t.Fatalf("StartGazeboSensorContainer() error = %v", err)
	}
	if len(runner.calls) != 2 {
		t.Fatalf("calls = %#v, want remove + run", runner.calls)
	}
	if !strings.Contains(runner.calls[0], "docker rm -f "+GazeboSensorContainer) {
		t.Fatalf("first call = %q", runner.calls[0])
	}
	if !strings.Contains(runner.calls[1], "docker run") {
		t.Fatalf("second call = %q", runner.calls[1])
	}
}

func TestTopicInfoParsers(t *testing.T) {
	output := `
Publisher count: 1
Node name: talker
Node namespace: /
Subscription count: 2
Node name: listener_a
Node namespace: /
Node name: listener_b
Node namespace: /
`
	publishers := PublisherNodesFromTopicInfo(output)
	subscribers := SubscriptionNodesFromTopicInfo(output)
	if len(publishers) != 1 || publishers[0] != "talker" {
		t.Fatalf("publishers = %#v", publishers)
	}
	if len(subscribers) != 2 || subscribers[1] != "listener_b" {
		t.Fatalf("subscribers = %#v", subscribers)
	}
	if TopicInfoArtifactName("/navlab/x2/status") != "topic_info_navlab_x2_status.txt" {
		t.Fatalf("unexpected artifact name")
	}
}
