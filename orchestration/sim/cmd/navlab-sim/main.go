package main

import (
	"context"
	"fmt"
	"os"
	"path/filepath"
	"time"

	"github.com/spf13/cobra"

	"navlab/orchestration-sim/internal/artifacts"
	"navlab/orchestration-sim/internal/config"
	simimages "navlab/orchestration-sim/internal/images"
	simruntime "navlab/orchestration-sim/internal/runtime"
	"navlab/orchestration-sim/internal/tasks"
	"navlab/orchestration-sim/internal/tasks/helpers"
	"navlab/orchestration-sim/internal/ui"
)

const defaultConfigPath = "config.toml"

type appContext struct {
	configPath   string
	artifactRoot string
}

type preparedTaskRun struct {
	Project            config.ProjectConfig
	TaskConfig         config.TaskConfig
	TaskRuntimeConfig  config.TaskRuntimeConfig
	Plan               tasks.Plan
	Result             artifacts.DryRunResult
	GeneratedArtifacts []tasks.GeneratedRuntimeArtifact
	RuntimeSpecs       tasks.RuntimeSpecBundle
}

func main() {
	if err := newRootCommand().Execute(); err != nil {
		fmt.Fprintln(os.Stderr, ui.Error("error: "+err.Error()))
		os.Exit(1)
	}
}

func newRootCommand() *cobra.Command {
	ctx := &appContext{}
	root := &cobra.Command{
		Use:           "navlab-sim",
		Short:         "NavLab simulation orchestration control plane",
		SilenceUsage:  true,
		SilenceErrors: true,
	}
	root.PersistentFlags().StringVar(
		&ctx.configPath,
		"config",
		defaultConfigPath,
		"sim orchestration TOML config path",
	)
	root.PersistentFlags().StringVar(
		&ctx.artifactRoot,
		"artifact-root",
		"",
		"override sim artifact root",
	)
	root.AddCommand(
		newDoctorCommand(ctx),
		newTaskDoctorCommand(ctx, "hover"),
		newTaskDoctorCommand(ctx, "exploration"),
		newTaskDoctorCommand(ctx, "scan-robustness"),
		newListHelpersCommand(),
		newListTasksCommand(ctx),
		newShowTaskCommand(ctx),
		newRunCommand(ctx),
		newBuildCommand(ctx),
	)
	return root
}

func newTaskDoctorCommand(ctx *appContext, taskID string) *cobra.Command {
	return &cobra.Command{
		Use:   taskID + "-doctor",
		Short: "Run static doctor checks for the " + taskID + " simulation task",
		RunE: func(cmd *cobra.Command, args []string) error {
			return doctorTask(ctx.loader(), ctx.registry(), ctx.helperRegistry(), taskID, ctx.artifactRoot)
		},
	}
}

func (ctx *appContext) loader() config.Loader {
	return config.NewLoader(ctx.configPath)
}

func (ctx *appContext) registry() *tasks.Registry {
	return tasks.DefaultRegistry()
}

func (ctx *appContext) helperRegistry() *helpers.Registry {
	return helpers.DefaultRegistry()
}

func newDoctorCommand(ctx *appContext) *cobra.Command {
	return &cobra.Command{
		Use:   "doctor",
		Short: "Validate sim orchestration and task config loading",
		RunE: func(cmd *cobra.Command, args []string) error {
			return doctor(ctx.loader())
		},
	}
}

func newListTasksCommand(ctx *appContext) *cobra.Command {
	return &cobra.Command{
		Use:   "list-tasks",
		Short: "List registered simulation task configs",
		RunE: func(cmd *cobra.Command, args []string) error {
			return listTasks(ctx.loader())
		},
	}
}

func newListHelpersCommand() *cobra.Command {
	return &cobra.Command{
		Use:   "list-helpers",
		Short: "List migrated non-real Python task helpers",
		RunE: func(cmd *cobra.Command, args []string) error {
			return listHelpers(helpers.DefaultRegistry())
		},
	}
}

func newShowTaskCommand(ctx *appContext) *cobra.Command {
	return &cobra.Command{
		Use:   "show-task <task-id>",
		Short: "Show one simulation task config",
		Args:  cobra.ExactArgs(1),
		RunE: func(cmd *cobra.Command, args []string) error {
			return showTask(ctx.loader(), args[0])
		},
	}
}

func newRunCommand(ctx *appContext) *cobra.Command {
	var dryRun bool
	var durationSec float64
	var simulationProfile string
	cmd := &cobra.Command{
		Use:   "run <task-id>",
		Short: "Plan or run one simulation task",
		Args:  cobra.ExactArgs(1),
		RunE: func(cmd *cobra.Command, args []string) error {
			return runTask(
				ctx.loader(),
				ctx.registry(),
				ctx.helperRegistry(),
				args[0],
				dryRun,
				ctx.artifactRoot,
				tasks.PlanOptions{
					DurationSec:       durationSec,
					SimulationProfile: simulationProfile,
				},
			)
		},
	}
	cmd.Flags().BoolVar(&dryRun, "dry-run", false, "print the task plan without starting runtime services")
	cmd.Flags().Float64Var(&durationSec, "duration-sec", 0, "override task duration in seconds")
	cmd.Flags().StringVar(&simulationProfile, "simulation-profile", "", "override simulation profile")
	return cmd
}

func newBuildCommand(ctx *appContext) *cobra.Command {
	var tag string
	var dryRun bool
	cmd := &cobra.Command{
		Use:   "build <companion|slam|gazebo-sensor|official-baseline|all>",
		Short: "Build NavLab simulation Docker images",
		Args:  cobra.ExactArgs(1),
		RunE: func(cmd *cobra.Command, args []string) error {
			return buildImages(ctx.loader(), args[0], tag, dryRun)
		},
	}
	cmd.Flags().StringVar(&tag, "tag", "", "override configured NavLab image tag strategy")
	cmd.Flags().BoolVar(&dryRun, "dry-run", false, "print docker build commands without running them")
	return cmd
}

func doctor(loader config.Loader) error {
	project, err := loader.LoadProject()
	if err != nil {
		return fmt.Errorf("config doctor failed: %w", err)
	}
	taskConfigs, err := loader.LoadTasks(project)
	if err != nil {
		return fmt.Errorf("task config doctor failed: %w", err)
	}
	configured, err := tasks.DefaultRegistry().Configure(taskConfigs)
	if err != nil {
		return fmt.Errorf("task registry doctor failed: %w", err)
	}
	fmt.Println(ui.Title("NavLab Sim Doctor"))
	fmt.Println(ui.StatusOK("config loaded"))
	fmt.Println(ui.StatusOK("task registry configured"))
	fmt.Println(ui.KeyValue("orchestration_family", project.Orchestration.Family))
	fmt.Println(ui.KeyValue("implementation", project.Orchestration.Implementation))
	fmt.Println(ui.KeyValue("runtime_mode", project.Runtime.Mode))
	fmt.Println(ui.KeyValue("backend", project.Runtime.Backend))
	fmt.Println(ui.KeyValue("task_config_format", "yaml"))
	fmt.Println(ui.KeyValue("task_count", len(configured)))
	return nil
}

func buildImages(loader config.Loader, kind string, tag string, dryRun bool) error {
	project, err := loader.LoadProject()
	if err != nil {
		return fmt.Errorf("failed to load config: %w", err)
	}
	if err := resolveWorkspaceRoot(loader, &project); err != nil {
		return fmt.Errorf("failed to resolve workspace root: %w", err)
	}
	result, err := simimages.Build(context.Background(), project, simimages.BuildOptions{
		Kind:   kind,
		Tag:    tag,
		DryRun: dryRun,
		Stdout: os.Stdout,
		Stderr: os.Stderr,
	})
	if err != nil {
		return err
	}
	fmt.Println(ui.Title("NavLab Sim Image Build"))
	fmt.Println(ui.KeyValue("dry_run", result.DryRun))
	fmt.Println(ui.KeyValue("kind", kind))
	if tag != "" {
		fmt.Println(ui.KeyValue("tag_override", tag))
	}
	for _, spec := range result.Specs {
		fmt.Println(ui.Subtitle(spec.Kind))
		fmt.Println(ui.KeyValue("context", spec.Context))
		fmt.Println(ui.KeyValue("dockerfile", spec.Dockerfile))
		fmt.Println(ui.KeyValue("target", spec.Target))
		fmt.Println(ui.KeyValue("image", spec.Image))
		fmt.Println(ui.KeyValue("command", spec.Command))
	}
	if !dryRun {
		fmt.Println(ui.StatusOK("NavLab image build completed"))
	}
	return nil
}

func listTasks(loader config.Loader) error {
	project, err := loader.LoadProject()
	if err != nil {
		return fmt.Errorf("failed to load config: %w", err)
	}
	taskConfigs, err := loader.LoadTasks(project)
	if err != nil {
		return fmt.Errorf("failed to load task configs: %w", err)
	}
	configured, err := tasks.DefaultRegistry().Configure(taskConfigs)
	if err != nil {
		return fmt.Errorf("failed to configure task registry: %w", err)
	}
	fmt.Println(ui.Title("Simulation Tasks"))
	for _, task := range configured {
		fmt.Printf("%s\t%s\t%s\n", ui.TaskID(task.Config.ID), task.Config.Family, task.Config.Description)
	}
	return nil
}

func listHelpers(registry *helpers.Registry) error {
	fmt.Println(ui.Title("Simulation Helper Inventory"))
	for _, helper := range registry.List() {
		status := helper.MigrationStatus
		if helper.RuntimeAction {
			status += ",runtime"
		}
		fmt.Printf("%s\t%s\t%s\n", ui.TaskID(helper.ID), helper.Phase, status)
	}
	return nil
}

func showTask(loader config.Loader, taskID string) error {
	project, err := loader.LoadProject()
	if err != nil {
		return fmt.Errorf("failed to load config: %w", err)
	}
	task, err := loader.LoadTask(project, taskID)
	if err != nil {
		return fmt.Errorf("failed to load task %q: %w", taskID, err)
	}
	configured, err := tasks.DefaultRegistry().ConfigureOne(task)
	if err != nil {
		return fmt.Errorf("failed to configure task %q: %w", taskID, err)
	}
	fmt.Println(ui.Title("Simulation Task"))
	fmt.Println(ui.KeyValue("id", configured.Config.ID))
	fmt.Println(ui.KeyValue("family", configured.Config.Family))
	fmt.Println(ui.KeyValue("description", configured.Config.Description))
	fmt.Println(ui.KeyValue("duration_sec", fmt.Sprintf("%.3f", configured.Config.Task.DurationSec)))
	fmt.Println(ui.KeyValue("simulation_profile", configured.Config.Task.SimulationProfile))
	fmt.Println(ui.KeyValue("capabilities", configured.Config.Capabilities))
	helperDefinitions, err := helpers.DefaultRegistry().Resolve(configured.Definition.HelperIDs)
	if err != nil {
		return fmt.Errorf("failed to resolve helpers for task %q: %w", taskID, err)
	}
	fmt.Println(ui.KeyValue("plan_steps", configured.Definition.Steps))
	fmt.Println(ui.Subtitle("Helpers"))
	for _, helper := range helperDefinitions {
		fmt.Printf("%s\t%s\t%s\n", ui.TaskID(helper.ID), helper.Phase, helper.Role)
	}
	return nil
}

func runTask(
	loader config.Loader,
	registry *tasks.Registry,
	helperRegistry *helpers.Registry,
	taskID string,
	dryRun bool,
	artifactRootOverride string,
	options tasks.PlanOptions,
) error {
	prepared, err := prepareTaskRun(loader, registry, helperRegistry, taskID, dryRun, artifactRootOverride, options)
	if err != nil {
		return err
	}
	if !dryRun {
		return runLiveTask(prepared.Project, prepared.TaskRuntimeConfig, prepared.Plan, prepared.Result, prepared.GeneratedArtifacts, prepared.RuntimeSpecs)
	}
	fmt.Println(ui.Title("Simulation Task Plan"))
	fmt.Println(ui.KeyValue("dry_run", true))
	fmt.Println(ui.KeyValue("run_id", prepared.Result.RunID))
	fmt.Println(ui.KeyValue("id", prepared.Plan.TaskID))
	fmt.Println(ui.KeyValue("description", prepared.Plan.Description))
	fmt.Println(ui.KeyValue("duration_sec", fmt.Sprintf("%.3f", prepared.Plan.DurationSec)))
	fmt.Println(ui.KeyValue("simulation_profile", prepared.Plan.SimulationProfile))
	fmt.Println(ui.KeyValue("capabilities", prepared.Plan.Capabilities))
	fmt.Println(ui.KeyValue("artifact_dir", prepared.Result.ArtifactDir))
	fmt.Println(ui.KeyValue("task_plan", prepared.Result.PlanPath))
	fmt.Println(ui.KeyValue("manifest", prepared.Result.ManifestPath))
	fmt.Println(ui.KeyValue("generated_runtime_artifacts", len(prepared.GeneratedArtifacts)))
	fmt.Println(ui.KeyValue("runtime_services", len(prepared.RuntimeSpecs.Services)))
	fmt.Println(ui.KeyValue("runtime_probes", len(prepared.RuntimeSpecs.Probes)))
	fmt.Println(ui.KeyValue("runtime_rosbags", len(prepared.RuntimeSpecs.Rosbags)))
	fmt.Println(ui.Subtitle("Helpers"))
	for _, helper := range prepared.Plan.Helpers {
		status := helper.MigrationStatus
		if helper.RuntimeAction {
			status += ",runtime"
		}
		fmt.Printf("%s %s %s\n", ui.TaskID(helper.ID), ui.Muted(helper.Phase), status)
	}
	fmt.Println(ui.Subtitle("Steps"))
	for index, step := range prepared.Plan.Steps {
		fmt.Printf("%s %s\n", ui.StepNumber(index+1), step)
	}
	return nil
}

func doctorTask(
	loader config.Loader,
	registry *tasks.Registry,
	helperRegistry *helpers.Registry,
	taskID string,
	artifactRootOverride string,
) error {
	prepared, err := prepareTaskRun(loader, registry, helperRegistry, taskID, true, artifactRootOverride, tasks.PlanOptions{})
	if err != nil {
		return err
	}
	summary := tasks.BuildStaticDoctorSummary(prepared.Project, prepared.Plan, prepared.Result.RunID, prepared.Result.ArtifactDir, prepared.GeneratedArtifacts, prepared.RuntimeSpecs)
	summaryPath := filepath.Join(prepared.Result.ArtifactDir, "doctor_summary.json")
	if err := artifacts.WriteJSONArtifact(summaryPath, summary); err != nil {
		return fmt.Errorf("failed to write %s doctor summary: %w", taskID, err)
	}
	if err := artifacts.AppendManifestArtifacts(prepared.Result.ManifestPath, prepared.Result.ArtifactDir, []artifacts.GeneratedArtifact{
		{Type: "doctor_summary", Path: summaryPath},
	}); err != nil {
		return fmt.Errorf("failed to update %s doctor manifest: %w", taskID, err)
	}
	fmt.Println(ui.Title("Simulation Task Doctor"))
	fmt.Println(ui.KeyValue("id", taskID))
	fmt.Println(ui.KeyValue("status", "ok"))
	fmt.Println(ui.KeyValue("summary", summaryPath))
	fmt.Println(ui.KeyValue("generated_runtime_artifacts", len(prepared.GeneratedArtifacts)))
	fmt.Println(ui.KeyValue("runtime_services", len(prepared.RuntimeSpecs.Services)))
	fmt.Println(ui.KeyValue("runtime_probes", len(prepared.RuntimeSpecs.Probes)))
	fmt.Println(ui.KeyValue("runtime_rosbags", len(prepared.RuntimeSpecs.Rosbags)))
	return nil
}

func prepareTaskRun(
	loader config.Loader,
	registry *tasks.Registry,
	helperRegistry *helpers.Registry,
	taskID string,
	dryRun bool,
	artifactRootOverride string,
	options tasks.PlanOptions,
) (preparedTaskRun, error) {
	project, err := loader.LoadProject()
	if err != nil {
		return preparedTaskRun{}, fmt.Errorf("failed to load config: %w", err)
	}
	if err := resolveWorkspaceRoot(loader, &project); err != nil {
		return preparedTaskRun{}, fmt.Errorf("failed to resolve workspace root: %w", err)
	}
	taskConfig, err := loader.LoadTask(project, taskID)
	if err != nil {
		return preparedTaskRun{}, fmt.Errorf("failed to load task %q: %w", taskID, err)
	}
	task, err := registry.ConfigureOne(taskConfig)
	if err != nil {
		return preparedTaskRun{}, fmt.Errorf("failed to configure task %q: %w", taskID, err)
	}
	plan, err := task.Plan(options, helperRegistry)
	if err != nil {
		return preparedTaskRun{}, fmt.Errorf("failed to build task plan for %q: %w", taskID, err)
	}
	taskRuntimeConfig, err := config.BuildTaskRuntimeConfig(project, taskConfig)
	if err != nil {
		return preparedTaskRun{}, fmt.Errorf("failed to build runtime config for %q: %w", taskID, err)
	}
	plan.Execution.TaskParameters["runtime_config"] = taskRuntimeConfig
	artifactRootPath := project.Paths.ArtifactRoot
	if artifactRootOverride != "" {
		artifactRootPath = artifactRootOverride
	}
	artifactRoot, err := loader.ResolveProjectPath(artifactRootPath)
	if err != nil {
		return preparedTaskRun{}, fmt.Errorf("failed to resolve artifact root: %w", err)
	}
	result, err := artifacts.NewWriter(artifactRoot).WriteRunPlan(project, plan, time.Now(), artifacts.RunPlanOptions{DryRun: dryRun})
	if err != nil {
		return preparedTaskRun{}, fmt.Errorf("failed to write run artifacts: %w", err)
	}
	generatedArtifacts, err := tasks.GenerateRuntimeArtifacts(project, plan, taskRuntimeConfig, result.ArtifactDir)
	if err != nil {
		return preparedTaskRun{}, fmt.Errorf("failed to generate runtime artifacts for %q: %w", taskID, err)
	}
	if err := artifacts.AppendManifestArtifacts(result.ManifestPath, result.ArtifactDir, manifestArtifacts(generatedArtifacts)); err != nil {
		return preparedTaskRun{}, fmt.Errorf("failed to update run manifest for %q: %w", taskID, err)
	}
	runtimeSpecs, err := tasks.BuildRuntimeSpecs(project, plan.Execution, result.ArtifactDir)
	if err != nil {
		return preparedTaskRun{}, fmt.Errorf("failed to validate runtime specs for %q: %w", taskID, err)
	}
	runtimePlan, err := tasks.BuildRuntimePlanContract(plan, result.RunID, runtimeSpecs)
	if err != nil {
		return preparedTaskRun{}, fmt.Errorf("failed to build runtime plan contract for %q: %w", taskID, err)
	}
	runtimePlanPath := filepath.Join(result.ArtifactDir, "runtime_plan.json")
	if err := artifacts.WriteJSONArtifact(runtimePlanPath, runtimePlan); err != nil {
		return preparedTaskRun{}, fmt.Errorf("failed to write runtime plan contract for %q: %w", taskID, err)
	}
	if err := artifacts.AppendManifestArtifacts(result.ManifestPath, result.ArtifactDir, []artifacts.GeneratedArtifact{
		{Type: "runtime_plan", Path: runtimePlanPath},
	}); err != nil {
		return preparedTaskRun{}, fmt.Errorf("failed to update runtime plan manifest for %q: %w", taskID, err)
	}
	return preparedTaskRun{
		Project:            project,
		TaskConfig:         taskConfig,
		TaskRuntimeConfig:  taskRuntimeConfig,
		Plan:               plan,
		Result:             result,
		GeneratedArtifacts: generatedArtifacts,
		RuntimeSpecs:       runtimeSpecs,
	}, nil
}

func runLiveTask(
	project config.ProjectConfig,
	taskRuntimeConfig config.TaskRuntimeConfig,
	plan tasks.Plan,
	result artifacts.DryRunResult,
	generatedArtifacts []tasks.GeneratedRuntimeArtifact,
	runtimeSpecs tasks.RuntimeSpecBundle,
) error {
	execution, executionErr := tasks.ExecuteRuntimeSpecs(
		simruntime.NewDockerBackend(nil),
		runtimeSpecs,
		tasks.RuntimeExecutionOptions{WaitForRosbags: true},
	)
	summary := tasks.BuildLiveRunSummary(project, taskRuntimeConfig, plan, result.RunID, result.ArtifactDir, generatedArtifacts, runtimeSpecs, execution, executionErr)
	summaryPath := filepath.Join(result.ArtifactDir, "summary.json")
	summary.SummaryPath = summaryPath
	if err := artifacts.WriteJSONArtifact(summaryPath, summary); err != nil {
		return fmt.Errorf("failed to write live summary for %q: %w", plan.TaskID, err)
	}
	finalArtifacts, err := artifacts.FinalizeRunArtifacts(project, plan, result, stageLabel(plan.TaskID), controlMode(plan.TaskID, plan.SimulationProfile))
	if err != nil {
		return fmt.Errorf("failed to finalize live artifacts for %q: %w", plan.TaskID, err)
	}
	manifestEntries := []artifacts.GeneratedArtifact{
		{Type: "summary", Path: summaryPath},
	}
	manifestEntries = append(manifestEntries, finalArtifacts...)
	if err := artifacts.AppendManifestArtifacts(result.ManifestPath, result.ArtifactDir, manifestEntries); err != nil {
		return fmt.Errorf("failed to update live manifest for %q: %w", plan.TaskID, err)
	}
	fmt.Println(ui.Title("Simulation Task Run"))
	fmt.Println(ui.KeyValue("dry_run", false))
	fmt.Println(ui.KeyValue("run_id", result.RunID))
	fmt.Println(ui.KeyValue("id", plan.TaskID))
	fmt.Println(ui.KeyValue("summary", summaryPath))
	fmt.Println(ui.KeyValue("artifact_dir", result.ArtifactDir))
	fmt.Println(ui.KeyValue("runtime_services", len(runtimeSpecs.Services)))
	fmt.Println(ui.KeyValue("runtime_probes", len(runtimeSpecs.Probes)))
	fmt.Println(ui.KeyValue("runtime_rosbags", len(runtimeSpecs.Rosbags)))
	if !summary.OK {
		fmt.Println(ui.KeyValue("status", "blocked"))
		if executionErr != nil {
			return fmt.Errorf("task %q live run failed; summary: %s: %w", plan.TaskID, summaryPath, executionErr)
		}
		return fmt.Errorf("task %q live run blocked; summary: %s", plan.TaskID, summaryPath)
	}
	fmt.Println(ui.KeyValue("status", "ok"))
	return nil
}

func stageLabel(taskID string) string {
	switch taskID {
	case "hover":
		return "Go sim FCU/SLAM hover + landing acceptance"
	case "exploration":
		return "Go sim official-maze exploration acceptance"
	case "scan-robustness":
		return "Go sim scan robustness acceptance"
	default:
		return "Go sim task acceptance"
	}
}

func controlMode(taskID string, profile string) string {
	if profile == "" {
		profile = "default"
	}
	return taskID + "_" + profile
}

func resolveWorkspaceRoot(loader config.Loader, project *config.ProjectConfig) error {
	if project.Paths.WorkspaceRoot == "" {
		return nil
	}
	resolved, err := loader.ResolveProjectPath(project.Paths.WorkspaceRoot)
	if err != nil {
		return err
	}
	project.Paths.WorkspaceRoot = resolved
	return nil
}

func manifestArtifacts(generated []tasks.GeneratedRuntimeArtifact) []artifacts.GeneratedArtifact {
	converted := make([]artifacts.GeneratedArtifact, 0, len(generated))
	for _, artifact := range generated {
		converted = append(converted, artifacts.GeneratedArtifact{
			Type: artifact.Type,
			Path: artifact.Path,
		})
	}
	return converted
}
