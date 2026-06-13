package images

import (
	"bytes"
	"context"
	"errors"
	"io"
	"reflect"
	"strings"
	"testing"

	"navlab/orchestration-sim/internal/config"
)

type recordingRunner struct {
	calls []string
	err   error
}

func (runner *recordingRunner) Run(ctx context.Context, name string, args []string, stdout io.Writer, stderr io.Writer) error {
	runner.calls = append(runner.calls, name+" "+strings.Join(args, " "))
	return runner.err
}

func TestResolveBuildSpecsAllUsesConfiguredImages(t *testing.T) {
	t.Setenv("NAVLAB_SIM_DISTRO", "")
	project := testProject()

	specs, err := ResolveBuildSpecs(project, KindAll, "test-tag")
	if err != nil {
		t.Fatal(err)
	}
	if len(specs) != 9 {
		t.Fatalf("specs len = %d", len(specs))
	}
	kinds := make([]string, 0, len(specs))
	for _, spec := range specs {
		kinds = append(kinds, spec.Kind)
	}
	wantKinds := []string{
		KindRosBase, KindArduPilotSITL, KindMAVLinkRouter, KindGazeboHeadless, KindFastLIO,
		KindCompanion, KindSlam, KindGazeboSensor, KindOfficialBaseline,
	}
	if !reflect.DeepEqual(kinds, wantKinds) {
		t.Fatalf("kinds = %#v", kinds)
	}
	first := specs[0]
	if first.Image != "navlab/ros-base:test-tag" || first.Group != KindBase {
		t.Fatalf("image = %q", first.Image)
	}
	wantCommand := []string{
		"docker", "build",
		"-f", "/workspace/docker/images/base/ros-base.Dockerfile",
		"--build-arg", "INFRA_TAG=test-tag",
		"--build-arg", "ROS_DISTRO=humble",
		"-t", "navlab/ros-base:test-tag",
		"/workspace",
	}
	if !reflect.DeepEqual(first.Command, wantCommand) {
		t.Fatalf("command = %#v", first.Command)
	}
}

func TestResolveBuildSpecsRuntimeGroupUsesRuntimeImages(t *testing.T) {
	t.Setenv("NAVLAB_SIM_DISTRO", "")
	specs, err := ResolveBuildSpecs(testProject(), KindRuntime, "local")
	if err != nil {
		t.Fatal(err)
	}
	kinds := []string{specs[0].Kind, specs[1].Kind, specs[2].Kind, specs[3].Kind}
	if !reflect.DeepEqual(kinds, []string{KindCompanion, KindSlam, KindGazeboSensor, KindOfficialBaseline}) {
		t.Fatalf("kinds = %#v", kinds)
	}
	if specs[0].Group != GroupRuntime || specs[0].Dockerfile != "/workspace/docker/images/runtime/companion.Dockerfile" {
		t.Fatalf("runtime spec = %#v", specs[0])
	}
}

func TestResolveBuildSpecsInfraGroupExcludesBase(t *testing.T) {
	t.Setenv("NAVLAB_SIM_DISTRO", "")
	specs, err := ResolveBuildSpecs(testProject(), KindInfra, "local")
	if err != nil {
		t.Fatal(err)
	}
	kinds := make([]string, 0, len(specs))
	for _, spec := range specs {
		kinds = append(kinds, spec.Kind)
	}
	wantKinds := []string{KindArduPilotSITL, KindMAVLinkRouter, KindGazeboHeadless, KindFastLIO}
	if !reflect.DeepEqual(kinds, wantKinds) {
		t.Fatalf("kinds = %#v", kinds)
	}
}

func TestResolveBuildSpecsSingleInfraImage(t *testing.T) {
	t.Setenv("NAVLAB_SIM_DISTRO", "")
	specs, err := ResolveBuildSpecsWithOptions(testProject(), BuildOptions{
		Kind:  KindInfra,
		Image: KindGazeboHeadless,
		Tag:   "local",
	})
	if err != nil {
		t.Fatal(err)
	}
	if len(specs) != 1 || specs[0].Kind != KindGazeboHeadless || specs[0].Group != GroupInfra {
		t.Fatalf("specs = %#v", specs)
	}
	if specs[0].Image != "navlab/gazebo-headless:local" {
		t.Fatalf("image = %q", specs[0].Image)
	}
}

func TestResolveBuildSpecsBaseGroupBuildsRosBase(t *testing.T) {
	t.Setenv("NAVLAB_SIM_DISTRO", "")
	specs, err := ResolveBuildSpecsWithOptions(testProject(), BuildOptions{Kind: KindBase, Tag: "local"})
	if err != nil {
		t.Fatal(err)
	}
	if len(specs) != 1 || specs[0].ConfigKey != "ros_base" || specs[0].Image != "navlab/ros-base:local" {
		t.Fatalf("specs = %#v", specs)
	}
}

func TestResolveBuildSpecsRejectsImageWithBaseGroup(t *testing.T) {
	t.Setenv("NAVLAB_SIM_DISTRO", "")
	_, err := ResolveBuildSpecsWithOptions(testProject(), BuildOptions{
		Kind:  KindBase,
		Image: KindRosBase,
		Tag:   "local",
	})
	if err == nil || !strings.Contains(err.Error(), "--image can only be used with infra or runtime") {
		t.Fatalf("err = %v", err)
	}
}

func TestResolveBuildSpecsRejectsImageAsBuildGroup(t *testing.T) {
	t.Setenv("NAVLAB_SIM_DISTRO", "")
	_, err := ResolveBuildSpecs(testProject(), KindRosBase, "local")
	if err == nil || !strings.Contains(err.Error(), "invalid image group") {
		t.Fatalf("err = %v", err)
	}
}

func TestResolveBuildSpecsRejectsImageFromWrongGroup(t *testing.T) {
	t.Setenv("NAVLAB_SIM_DISTRO", "")
	_, err := ResolveBuildSpecsWithOptions(testProject(), BuildOptions{
		Kind:  KindInfra,
		Image: KindCompanion,
		Tag:   "local",
	})
	if err == nil || !strings.Contains(err.Error(), "belongs to runtime, not infra") {
		t.Fatalf("err = %v", err)
	}
}

func TestResolveBuildSpecsRejectsBaseImageFromInfraGroup(t *testing.T) {
	t.Setenv("NAVLAB_SIM_DISTRO", "")
	_, err := ResolveBuildSpecsWithOptions(testProject(), BuildOptions{
		Kind:  KindInfra,
		Image: KindRosBase,
		Tag:   "local",
	})
	if err == nil || !strings.Contains(err.Error(), "belongs to base, not infra") {
		t.Fatalf("err = %v", err)
	}
}

func TestResolveBuildSpecsUsesEnvDistroAndFallback(t *testing.T) {
	t.Setenv("NAVLAB_SIM_DISTRO", "")
	project := testProject()
	specs, err := ResolveBuildSpecsWithOptions(project, BuildOptions{Kind: KindBase, Tag: "local"})
	if err != nil {
		t.Fatal(err)
	}
	if specs[0].Distro != "humble" || !strings.Contains(strings.Join(specs[0].Command, " "), "ROS_DISTRO=humble") {
		t.Fatalf("spec = %#v", specs[0])
	}

	t.Setenv("NAVLAB_SIM_DISTRO", "jazzy")
	specs, err = ResolveBuildSpecsWithOptions(project, BuildOptions{Kind: KindBase, Tag: "local"})
	if err != nil {
		t.Fatal(err)
	}
	if specs[0].Distro != "jazzy" || !strings.Contains(strings.Join(specs[0].Command, " "), "ROS_DISTRO=jazzy") {
		t.Fatalf("spec = %#v", specs[0])
	}
}

func TestResolveBuildSpecsRejectsUnsupportedDistro(t *testing.T) {
	t.Setenv("NAVLAB_SIM_DISTRO", "iron")
	_, err := ResolveBuildSpecsWithOptions(testProject(), BuildOptions{Kind: KindBase, Tag: "local"})
	if err == nil || !strings.Contains(err.Error(), "unsupported ROS distro") {
		t.Fatalf("err = %v", err)
	}
}

func TestResolveBuildSpecsRejectsUnsupportedTagPolicy(t *testing.T) {
	t.Setenv("NAVLAB_SIM_DISTRO", "")
	project := testProject()
	project.Navlab.Images.TagPolicy = "latest"
	_, err := ResolveBuildSpecsWithOptions(project, BuildOptions{Kind: KindBase})
	if err == nil || !strings.Contains(err.Error(), "expected distro-git-commit, distro-datetime, or distro-latest") {
		t.Fatalf("err = %v", err)
	}
}

func TestBuildDryRunDoesNotRunDocker(t *testing.T) {
	t.Setenv("NAVLAB_SIM_DISTRO", "")
	runner := &recordingRunner{}
	result, err := Build(context.Background(), testProject(), BuildOptions{
		Kind:   KindRuntime,
		Image:  KindSlam,
		Tag:    "local",
		DryRun: true,
		Runner: runner,
	})
	if err != nil {
		t.Fatal(err)
	}
	if len(result.Specs) != 1 || result.Specs[0].Kind != KindSlam {
		t.Fatalf("result = %#v", result)
	}
	if len(runner.calls) != 0 {
		t.Fatalf("runner calls = %#v", runner.calls)
	}
}

func TestBuildRunsDockerCommands(t *testing.T) {
	t.Setenv("NAVLAB_SIM_DISTRO", "")
	runner := &recordingRunner{}
	var stdout bytes.Buffer
	_, err := Build(context.Background(), testProject(), BuildOptions{
		Kind:   KindRuntime,
		Image:  KindGazeboSensor,
		Tag:    "local",
		Runner: runner,
		Stdout: &stdout,
	})
	if err != nil {
		t.Fatal(err)
	}
	if len(runner.calls) != 1 {
		t.Fatalf("runner calls = %#v", runner.calls)
	}
	if !strings.Contains(runner.calls[0], "docker build") || !strings.Contains(runner.calls[0], "navlab/gazebo-sensor:local") {
		t.Fatalf("runner call = %q", runner.calls[0])
	}
	if !strings.Contains(stdout.String(), "Building NavLab gazebo-sensor image") {
		t.Fatalf("stdout = %q", stdout.String())
	}
}

func TestBuildReturnsRunnerError(t *testing.T) {
	t.Setenv("NAVLAB_SIM_DISTRO", "")
	runner := &recordingRunner{err: errors.New("docker failed")}
	_, err := Build(context.Background(), testProject(), BuildOptions{
		Kind:   KindRuntime,
		Image:  KindCompanion,
		Runner: runner,
	})
	if err == nil || !strings.Contains(err.Error(), "docker failed") {
		t.Fatalf("err = %v", err)
	}
}

func TestResolveBuildSpecsRejectsInvalidKind(t *testing.T) {
	t.Setenv("NAVLAB_SIM_DISTRO", "")
	_, err := ResolveBuildSpecs(testProject(), "bad", "")
	if err == nil || !strings.Contains(err.Error(), "invalid image group") {
		t.Fatalf("err = %v", err)
	}
}

func testProject() config.ProjectConfig {
	return config.ProjectConfig{
		Paths: config.PathConfig{WorkspaceRoot: "/workspace"},
		Navlab: config.NavlabConfig{Images: config.ImageCatalog{
			TagPolicy: "distro-latest",
		}},
		Images: map[string]config.Image{
			"ros_base": {
				Group:      KindBase,
				Repository: "navlab/ros-base",
				Dockerfile: "docker/images/base/ros-base.Dockerfile",
				Context:    ".",
			},
			"ardupilot_sitl": {
				Group:      GroupInfra,
				Repository: "navlab/ardupilot-sitl",
				Dockerfile: "docker/images/infra/ardupilot-sitl.Dockerfile",
				Context:    ".",
			},
			"mavlink_router": {
				Group:      GroupInfra,
				Repository: "navlab/mavlink-router",
				Dockerfile: "docker/images/infra/mavlink-router.Dockerfile",
				Context:    ".",
			},
			"gazebo_headless": {
				Group:      GroupInfra,
				Repository: "navlab/gazebo-headless",
				Dockerfile: "docker/images/infra/gazebo-headless.Dockerfile",
				Context:    ".",
			},
			"fast_lio": {
				Group:      GroupInfra,
				Repository: "navlab/fast-lio",
				Dockerfile: "docker/images/infra/fast-lio.Dockerfile",
				Context:    ".",
			},
			"companion": {
				Group:      GroupRuntime,
				Repository: "navlab/companion",
				Dockerfile: "docker/images/runtime/companion.Dockerfile",
				Context:    ".",
				Target:     "navlab-companion",
			},
			"slam": {
				Group:      GroupRuntime,
				Repository: "navlab/slam-cartographer",
				Dockerfile: "docker/images/runtime/slam.Dockerfile",
				Context:    ".",
				Target:     "navlab-slam-cartographer",
			},
			"gazebo_sensor": {
				Group:      GroupRuntime,
				Repository: "navlab/gazebo-sensor",
				Dockerfile: "docker/images/runtime/gazebo-sensor.Dockerfile",
				Context:    ".",
				Target:     "navlab-gazebo-sensor",
			},
			"official_baseline": {
				Group:      GroupRuntime,
				Repository: "navlab/official-baseline",
				Dockerfile: "docker/images/runtime/official-baseline.Dockerfile",
				Context:    ".",
				Target:     "navlab-official-baseline",
			},
		},
	}
}
