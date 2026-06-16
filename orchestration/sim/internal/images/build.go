package images

import (
	"bytes"
	"context"
	"errors"
	"fmt"
	"io"
	"os"
	"os/exec"
	"path/filepath"
	"sort"
	"strings"
	"time"

	"navlab/orchestration-sim/internal/config"
)

const (
	GroupInfra   = "infra"
	GroupRuntime = "runtime"

	KindAll              = "all"
	KindInfra            = GroupInfra
	KindRuntime          = GroupRuntime
	KindBase             = "base"
	KindRosBase          = "ros-base"
	KindArduPilotSITL    = "ardupilot-sitl"
	KindMAVLinkRouter    = "mavlink-router"
	KindGazeboHeadless   = "gazebo-headless"
	KindFastLIO          = "fast-lio"
	KindCompanion        = "companion"
	KindSlam             = "slam"
	KindGazeboSensor     = "gazebo-sensor"
	KindOfficialBaseline = "official-baseline"
)

var (
	canonicalBaseKinds    = []string{KindRosBase}
	canonicalInfraKinds   = []string{KindArduPilotSITL, KindMAVLinkRouter, KindGazeboHeadless, KindFastLIO}
	canonicalRuntimeKinds = []string{KindCompanion, KindSlam, KindGazeboSensor, KindOfficialBaseline}
)

type BuildOptions struct {
	Kind   string
	Image  string
	Tag    string
	Distro string
	DryRun bool
	Stdout io.Writer
	Stderr io.Writer
	Runner CommandRunner
}

type BuildSpec struct {
	Kind       string   `json:"kind"`
	Group      string   `json:"group"`
	ConfigKey  string   `json:"config_key"`
	Distro     string   `json:"distro"`
	Tag        string   `json:"tag"`
	Context    string   `json:"context"`
	Dockerfile string   `json:"dockerfile"`
	Target     string   `json:"target"`
	Image      string   `json:"image"`
	Command    []string `json:"command"`
}

type BuildResult struct {
	DryRun bool        `json:"dry_run"`
	Specs  []BuildSpec `json:"specs"`
}

type CommandRunner interface {
	Run(ctx context.Context, name string, args []string, stdout io.Writer, stderr io.Writer) error
}

type ExecRunner struct{}

func (ExecRunner) Run(ctx context.Context, name string, args []string, stdout io.Writer, stderr io.Writer) error {
	cmd := exec.CommandContext(ctx, name, args...)
	cmd.Stdout = stdout
	cmd.Stderr = stderr
	return cmd.Run()
}

func Build(ctx context.Context, project config.ProjectConfig, options BuildOptions) (BuildResult, error) {
	specs, err := ResolveBuildSpecsWithOptions(project, options)
	if err != nil {
		return BuildResult{}, err
	}
	result := BuildResult{DryRun: options.DryRun, Specs: specs}
	if options.DryRun {
		return result, nil
	}
	runner := options.Runner
	if runner == nil {
		runner = ExecRunner{}
	}
	stdout := options.Stdout
	if stdout == nil {
		stdout = os.Stdout
	}
	stderr := options.Stderr
	if stderr == nil {
		stderr = os.Stderr
	}
	for _, spec := range specs {
		if _, err := fmt.Fprintf(stdout, "Building NavLab %s image %s\n", spec.Kind, spec.Image); err != nil {
			return result, err
		}
		if err := runner.Run(ctx, spec.Command[0], spec.Command[1:], stdout, stderr); err != nil {
			return result, fmt.Errorf("build %s image %s: %w", spec.Kind, spec.Image, err)
		}
	}
	return result, nil
}

func ResolveBuildSpecs(project config.ProjectConfig, kind string, tagOverride string) ([]BuildSpec, error) {
	return ResolveBuildSpecsWithOptions(project, BuildOptions{Kind: kind, Tag: tagOverride})
}

func ResolveBuildSpecsWithOptions(project config.ProjectConfig, options BuildOptions) ([]BuildSpec, error) {
	normalizedKind := strings.TrimSpace(options.Kind)
	if normalizedKind == "" {
		normalizedKind = KindAll
	}
	selected, err := selectKinds(project, normalizedKind, options.Image)
	if err != nil {
		return nil, err
	}
	specs := make([]BuildSpec, 0, len(selected))
	for _, selectedKind := range selected {
		configKey := configKeyForKind(selectedKind)
		image, ok := project.Images[configKey]
		if !ok {
			return nil, fmt.Errorf("navlab image %q is not configured", configKey)
		}
		spec, err := buildSpec(project, selectedKind, configKey, image, options.Tag, options.Distro)
		if err != nil {
			return nil, err
		}
		specs = append(specs, spec)
	}
	return specs, nil
}

func selectKinds(project config.ProjectConfig, kind string, image string) ([]string, error) {
	switch kind {
	case KindAll:
		if strings.TrimSpace(image) != "" {
			return nil, errors.New("--image can only be used with infra or runtime")
		}
		selected := append([]string(nil), canonicalBaseKinds...)
		selected = append(selected, canonicalInfraKinds...)
		selected = append(selected, canonicalRuntimeKinds...)
		return selected, nil
	case KindBase:
		if strings.TrimSpace(image) != "" {
			return nil, errors.New("--image can only be used with infra or runtime")
		}
		return []string{KindRosBase}, nil
	case KindInfra:
		if strings.TrimSpace(image) != "" {
			return selectImageInGroup(project, image, GroupInfra)
		}
		return append([]string(nil), canonicalInfraKinds...), nil
	case KindRuntime:
		if strings.TrimSpace(image) != "" {
			return selectImageInGroup(project, image, GroupRuntime)
		}
		return append([]string(nil), canonicalRuntimeKinds...), nil
	default:
		return nil, fmt.Errorf("invalid image group %q: expected base, infra, runtime, or all", kind)
	}
}

func selectImageInGroup(project config.ProjectConfig, image string, group string) ([]string, error) {
	kind := strings.TrimSpace(image)
	if kind == "" {
		return nil, errors.New("--image requires an image kind")
	}
	configKey := configKeyForKind(kind)
	imageConfig, ok := project.Images[configKey]
	if !ok {
		return nil, fmt.Errorf("image %q is not configured", kind)
	}
	if imageGroup(imageConfig) != group {
		return nil, fmt.Errorf("image %q belongs to %s, not %s", kind, imageGroup(imageConfig), group)
	}
	return []string{kind}, nil
}

func buildSpec(project config.ProjectConfig, kind string, configKey string, image config.Image, tagOverride string, distroOverride string) (BuildSpec, error) {
	if strings.TrimSpace(image.Repository) == "" {
		return BuildSpec{}, fmt.Errorf("navlab image %q repository is required", configKey)
	}
	contextPath, err := resolveWorkspacePath(project.Paths.WorkspaceRoot, image.Context)
	if err != nil {
		return BuildSpec{}, fmt.Errorf("resolve %s context: %w", configKey, err)
	}
	dockerfilePath, err := resolveWorkspacePath(project.Paths.WorkspaceRoot, image.Dockerfile)
	if err != nil {
		return BuildSpec{}, fmt.Errorf("resolve %s dockerfile: %w", configKey, err)
	}
	distro := resolveDistro(project, image, distroOverride)
	tag, err := ResolveImageTag(project, image, tagOverride, distro)
	if err != nil {
		return BuildSpec{}, err
	}
	fullImage := image.Repository + ":" + tag
	command := []string{
		"docker",
		"build",
		"-f", dockerfilePath,
	}
	if target := strings.TrimSpace(image.Target); target != "" {
		command = append(command, "--target", target)
	}
	for _, arg := range buildArgs(image, distro, tag) {
		command = append(command, "--build-arg", arg)
	}
	command = append(command,
		"-t", fullImage,
		contextPath,
	)
	return BuildSpec{
		Kind:       kind,
		Group:      imageGroup(image),
		ConfigKey:  configKey,
		Distro:     distro,
		Tag:        tag,
		Context:    contextPath,
		Dockerfile: dockerfilePath,
		Target:     strings.TrimSpace(image.Target),
		Image:      fullImage,
		Command:    command,
	}, nil
}

func resolveWorkspacePath(workspaceRoot string, value string) (string, error) {
	trimmed := strings.TrimSpace(value)
	if trimmed == "" {
		return "", errors.New("path is required")
	}
	if filepath.IsAbs(trimmed) {
		return filepath.Clean(trimmed), nil
	}
	root := strings.TrimSpace(workspaceRoot)
	if root == "" {
		root = "."
	}
	return filepath.Clean(filepath.Join(root, trimmed)), nil
}

func resolveDistro(project config.ProjectConfig, image config.Image, distroOverride string) string {
	if strings.TrimSpace(distroOverride) != "" {
		return normalizeDistro(strings.TrimSpace(distroOverride))
	}
	if envDistro := strings.TrimSpace(os.Getenv("NAVLAB_SIM_DISTRO")); envDistro != "" {
		return normalizeDistro(envDistro)
	}
	if strings.TrimSpace(image.Distro) != "" {
		return normalizeDistro(strings.TrimSpace(image.Distro))
	}
	if strings.TrimSpace(project.Navlab.Images.Distro) != "" {
		return normalizeDistro(strings.TrimSpace(project.Navlab.Images.Distro))
	}
	return "humble"
}

func ResolveImageTag(project config.ProjectConfig, image config.Image, tagOverride string, distroOverride string) (string, error) {
	distro := resolveDistro(project, image, distroOverride)
	if err := ValidateDistro(distro); err != nil {
		return "", err
	}
	if err := validateTagDistroCompatibility(tagOverride, distro); err != nil {
		return "", err
	}
	return resolveTag(project, image, tagOverride, distro)
}

func resolveTag(project config.ProjectConfig, image config.Image, tagOverride string, distro string) (string, error) {
	if strings.TrimSpace(tagOverride) != "" {
		return strings.TrimSpace(tagOverride), nil
	}
	policy := strings.TrimSpace(image.TagPolicy)
	if policy == "" {
		policy = strings.TrimSpace(project.Navlab.Images.TagPolicy)
	}
	if policy == "" {
		policy = strings.TrimSpace(project.Navlab.Images.TagStrategy)
	}
	if policy == "" {
		policy = "distro-git-commit"
	}
	switch strings.ToLower(policy) {
	case "distro-latest":
		return distro + "-latest", nil
	case "distro-git-commit":
		commit, err := gitCommitTag(project.Paths.WorkspaceRoot)
		if err != nil {
			return "", err
		}
		return distro + "-" + commit, nil
	case "distro-datetime":
		return distro + "-" + datetimeTag(), nil
	default:
		return "", fmt.Errorf("invalid navlab image tag_policy %q: expected distro-git-commit, distro-datetime, or distro-latest", policy)
	}
}

func validateTagDistroCompatibility(tagOverride string, distro string) error {
	tag := strings.TrimSpace(tagOverride)
	if tag == "" {
		return nil
	}
	for _, knownDistro := range []string{"humble", "jazzy"} {
		if strings.HasPrefix(tag, knownDistro+"-") && knownDistro != distro {
			return fmt.Errorf("image tag %q targets ROS distro %q but selected distro is %q", tag, knownDistro, distro)
		}
	}
	return nil
}

func gitCommitTag(workspaceRoot string) (string, error) {
	root := strings.TrimSpace(workspaceRoot)
	if root == "" {
		root = "."
	}
	cmd := exec.Command("git", "rev-parse", "--short=12", "HEAD")
	cmd.Dir = root
	var stdout bytes.Buffer
	cmd.Stdout = &stdout
	var stderr bytes.Buffer
	cmd.Stderr = &stderr
	if err := cmd.Run(); err != nil {
		detail := strings.TrimSpace(stderr.String())
		if detail != "" {
			return "", fmt.Errorf("could not resolve git commit image tag: %w: %s", err, detail)
		}
		return "", fmt.Errorf("could not resolve git commit image tag: %w", err)
	}
	tag := strings.TrimSpace(stdout.String())
	if tag == "" {
		return "", errors.New("could not resolve git commit image tag")
	}
	return tag, nil
}

func datetimeTag() string {
	return time.Now().UTC().Format("20060102T150405Z")
}

func buildArgs(image config.Image, distro string, tag string) []string {
	args := map[string]string{
		"ROS_DISTRO": distro,
		"INFRA_TAG":  tag,
	}
	for key, value := range image.BuildArgs {
		args[key] = value
	}
	keys := make([]string, 0, len(args))
	for key := range args {
		keys = append(keys, key)
	}
	sort.Strings(keys)
	values := make([]string, 0, len(keys))
	for _, key := range keys {
		values = append(values, key+"="+args[key])
	}
	return values
}

func imageGroup(image config.Image) string {
	group := strings.TrimSpace(image.Group)
	if group == "" {
		return GroupRuntime
	}
	return group
}

func normalizeDistro(distro string) string {
	return strings.ToLower(strings.TrimSpace(distro))
}

func ValidateDistro(distro string) error {
	switch normalizeDistro(distro) {
	case "humble", "jazzy":
		return nil
	default:
		return fmt.Errorf("unsupported ROS distro %q: expected humble or jazzy", distro)
	}
}

func configKeyForKind(kind string) string {
	switch kind {
	case KindRosBase:
		return "ros_base"
	case KindArduPilotSITL:
		return "ardupilot_sitl"
	case KindMAVLinkRouter:
		return "mavlink_router"
	case KindGazeboHeadless:
		return "gazebo_headless"
	case KindFastLIO:
		return "fast_lio"
	case KindGazeboSensor:
		return "gazebo_sensor"
	case KindOfficialBaseline:
		return "official_baseline"
	default:
		return strings.ReplaceAll(kind, "-", "_")
	}
}
