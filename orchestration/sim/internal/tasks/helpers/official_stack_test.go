package helpers

import (
	"strings"
	"testing"
)

func TestLuaExtractors(t *testing.T) {
	content := `
tracking_frame = "imu_link"
published_frame = "base_link"
use_odometry = false
`
	if value := ExtractLuaString(content, "tracking_frame"); value == nil || *value != "imu_link" {
		t.Fatalf("tracking_frame = %#v", value)
	}
	if value := ExtractLuaBool(content, "use_odometry"); value == nil || *value != false {
		t.Fatalf("use_odometry = %#v", value)
	}
}

func TestCartographerConfigSummaryFromContent(t *testing.T) {
	summary := CartographerConfigSummaryFromContent("config.lua", `odom_frame = "odom"`)
	if !summary.Present {
		t.Fatal("Present = false, want true")
	}
	if summary.Hash == "" {
		t.Fatal("Hash is empty")
	}
	if summary.OdomFrame == nil || *summary.OdomFrame != "odom" {
		t.Fatalf("OdomFrame = %#v", summary.OdomFrame)
	}
}

func TestDockerRosShellArgs(t *testing.T) {
	args, err := DockerRosShellArgs(DockerRosShellSpec{
		Image:        "image",
		ShellCommand: "ros2 topic list",
		Env: map[string]string{
			"ROS_DOMAIN_ID": "85",
		},
	})
	if err != nil {
		t.Fatalf("DockerRosShellArgs() error = %v", err)
	}
	joined := strings.Join(args, " ")
	for _, want := range []string{"run --rm", "--network host", "ROS_DOMAIN_ID=85", "image bash -lc ros2 topic list"} {
		if !strings.Contains(joined, want) {
			t.Fatalf("args missing %q:\n%s", want, joined)
		}
	}
}
