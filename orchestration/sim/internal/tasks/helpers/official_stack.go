package helpers

import (
	"crypto/sha256"
	"encoding/hex"
	"os"
	"regexp"
)

const CartographerConfigPath = "navlab/common/slam/ros/localization/navlab_cartographer_adapter/config/" + DiagnosticCartographerConfigBasename

var OfficialReferences = map[string]string{
	"ros2":          "https://ardupilot.org/dev/docs/ros2.html",
	"sitl":          "https://ardupilot.org/dev/docs/ros2-sitl.html",
	"gazebo":        "https://ardupilot.org/dev/docs/ros2-gazebo.html",
	"cartographer":  "https://ardupilot.org/dev/docs/ros2-cartographer-slam.html",
	"ardupilot_ros": "https://github.com/ArduPilot/ardupilot_ros",
	"ardupilot_gz":  "https://github.com/ArduPilot/ardupilot_gz",
}

var KnownExternalNavRoutes = []string{"official_dds", "mavlink_fallback", "diagnostic_only", "unknown"}

type CartographerConfigSummary struct {
	Present        bool    `json:"cartographer_config_present"`
	Path           string  `json:"cartographer_config_path"`
	Hash           string  `json:"cartographer_config_hash,omitempty"`
	UsesOdometry   *bool   `json:"cartographer_uses_odometry,omitempty"`
	TrackingFrame  *string `json:"tracking_frame,omitempty"`
	PublishedFrame *string `json:"published_frame,omitempty"`
	OdomFrame      *string `json:"odom_frame,omitempty"`
}

type DockerRosShellSpec struct {
	Image                  string
	ShellCommand           string
	Name                   string
	Network                string
	WorkspaceHostPath      string
	WorkspaceContainerPath string
	Env                    map[string]string
}

func ExtractLuaString(content string, key string) *string {
	pattern := regexp.MustCompile(`\b` + regexp.QuoteMeta(key) + `\s*=\s*"([^"]+)"`)
	match := pattern.FindStringSubmatch(content)
	if len(match) != 2 {
		return nil
	}
	value := match[1]
	return &value
}

func ExtractLuaBool(content string, key string) *bool {
	pattern := regexp.MustCompile(`\b` + regexp.QuoteMeta(key) + `\s*=\s*(true|false)`)
	match := pattern.FindStringSubmatch(content)
	if len(match) != 2 {
		return nil
	}
	value := match[1] == "true"
	return &value
}

func CartographerConfigSummaryFromContent(path string, content string) CartographerConfigSummary {
	sum := sha256.Sum256([]byte(content))
	return CartographerConfigSummary{
		Present:        true,
		Path:           path,
		Hash:           hex.EncodeToString(sum[:]),
		UsesOdometry:   ExtractLuaBool(content, "use_odometry"),
		TrackingFrame:  ExtractLuaString(content, "tracking_frame"),
		PublishedFrame: ExtractLuaString(content, "published_frame"),
		OdomFrame:      ExtractLuaString(content, "odom_frame"),
	}
}

func CartographerConfigSummaryFromFile(path string) (CartographerConfigSummary, error) {
	data, err := os.ReadFile(path)
	if err != nil {
		if os.IsNotExist(err) {
			return CartographerConfigSummary{Present: false, Path: path}, nil
		}
		return CartographerConfigSummary{}, err
	}
	return CartographerConfigSummaryFromContent(path, string(data)), nil
}

func DockerRosShellArgs(spec DockerRosShellSpec) ([]string, error) {
	if spec.Image == "" {
		return nil, nil
	}
	workspaceHostPath := spec.WorkspaceHostPath
	if workspaceHostPath == "" {
		workspaceHostPath = "."
	}
	workspaceContainerPath := spec.WorkspaceContainerPath
	if workspaceContainerPath == "" {
		workspaceContainerPath = "/workspace"
	}
	args := []string{
		"run",
		"--rm",
		"--network", valueOrDefault(spec.Network, "host"),
		"--volume", workspaceHostPath + ":" + workspaceContainerPath,
		"--workdir", workspaceContainerPath,
	}
	if spec.Name != "" {
		args = append(args, "--name", spec.Name)
	}
	for key, value := range spec.Env {
		args = append(args, "--env", key+"="+value)
	}
	args = append(args, spec.Image, "bash", "-lc", spec.ShellCommand)
	return args, nil
}

func valueOrDefault(value string, fallback string) string {
	if value == "" {
		return fallback
	}
	return value
}
