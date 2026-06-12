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
	"strings"

	"navlab/orchestration-sim/internal/config"
)

const (
	KindAll              = "all"
	KindCompanion        = "companion"
	KindSlam             = "slam"
	KindGazeboSensor     = "gazebo-sensor"
	KindOfficialBaseline = "official-baseline"
)

var canonicalKinds = []string{KindCompanion, KindSlam, KindGazeboSensor, KindOfficialBaseline}

type BuildOptions struct {
	Kind   string
	Tag    string
	DryRun bool
	Stdout io.Writer
	Stderr io.Writer
	Runner CommandRunner
}

type BuildSpec struct {
	Kind       string   `json:"kind"`
	ConfigKey  string   `json:"config_key"`
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
	specs, err := ResolveBuildSpecs(project, options.Kind, options.Tag)
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
	normalizedKind := strings.TrimSpace(kind)
	if normalizedKind == "" {
		normalizedKind = KindAll
	}
	selected, err := selectKinds(normalizedKind)
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
		spec, err := buildSpec(project, selectedKind, configKey, image, tagOverride)
		if err != nil {
			return nil, err
		}
		specs = append(specs, spec)
	}
	return specs, nil
}

func selectKinds(kind string) ([]string, error) {
	switch kind {
	case KindAll:
		return append([]string(nil), canonicalKinds...), nil
	case KindCompanion, KindSlam, KindGazeboSensor, KindOfficialBaseline:
		return []string{kind}, nil
	default:
		return nil, fmt.Errorf("invalid image kind %q: expected companion, slam, gazebo-sensor, official-baseline, or all", kind)
	}
}

func buildSpec(project config.ProjectConfig, kind string, configKey string, image config.Image, tagOverride string) (BuildSpec, error) {
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
	target := strings.TrimSpace(image.Target)
	if target == "" {
		return BuildSpec{}, fmt.Errorf("navlab image %q target is required", configKey)
	}
	tag, err := resolveTag(project, tagOverride)
	if err != nil {
		return BuildSpec{}, err
	}
	fullImage := image.Repository + ":" + tag
	command := []string{
		"docker",
		"build",
		"-f", dockerfilePath,
		"--target", target,
		"-t", fullImage,
		contextPath,
	}
	return BuildSpec{
		Kind:       kind,
		ConfigKey:  configKey,
		Context:    contextPath,
		Dockerfile: dockerfilePath,
		Target:     target,
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

func resolveTag(project config.ProjectConfig, tagOverride string) (string, error) {
	if strings.TrimSpace(tagOverride) != "" {
		return strings.TrimSpace(tagOverride), nil
	}
	strategy := strings.TrimSpace(project.Navlab.Images.TagStrategy)
	if strategy == "" {
		strategy = "latest"
	}
	switch strings.ToLower(strategy) {
	case "latest":
		return "latest", nil
	case "git-commit":
		return gitCommitTag(project.Paths.WorkspaceRoot)
	default:
		return "", fmt.Errorf("invalid navlab image tag_strategy %q: expected latest or git-commit", strategy)
	}
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

func configKeyForKind(kind string) string {
	switch kind {
	case KindGazeboSensor:
		return "gazebo_sensor"
	case KindOfficialBaseline:
		return "official_baseline"
	default:
		return kind
	}
}
