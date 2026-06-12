package runtime

import (
	"context"
	"fmt"
	"os/exec"
	"strconv"
	"time"
)

type CommandRunner interface {
	Run(ctx context.Context, command string, args ...string) (CommandResult, error)
}

type CommandResult struct {
	Stdout   string
	Stderr   string
	ExitCode int
}

type ExecRunner struct{}

func (ExecRunner) Run(ctx context.Context, command string, args ...string) (CommandResult, error) {
	cmd := exec.CommandContext(ctx, command, args...)
	stdoutStderr, err := cmd.CombinedOutput()
	exitCode := 0
	if err != nil {
		exitCode = 1
		if exitErr, ok := err.(*exec.ExitError); ok {
			exitCode = exitErr.ExitCode()
		}
	}
	return CommandResult{Stdout: string(stdoutStderr), ExitCode: exitCode}, err
}

type DockerBackend struct {
	Runner CommandRunner
	Now    func() time.Time
}

func NewDockerBackend(runner CommandRunner) DockerBackend {
	if runner == nil {
		runner = ExecRunner{}
	}
	return DockerBackend{
		Runner: runner,
		Now:    time.Now,
	}
}

func (backend DockerBackend) StartService(spec ServiceSpec) (RuntimeHandle, error) {
	if err := spec.ValidateDocker(); err != nil {
		return RuntimeHandle{}, err
	}
	args, err := DockerServiceArgs(spec)
	if err != nil {
		return RuntimeHandle{}, err
	}
	if spec.ContainerName != "" {
		_, _ = backend.runner().Run(context.Background(), "docker", "rm", "-f", spec.ContainerName)
	}
	result, err := backend.runner().Run(context.Background(), "docker", args...)
	if err != nil {
		_ = writeLog(spec.LogPath, result.Stdout, result.Stderr)
		return RuntimeHandle{}, fmt.Errorf("docker service %s failed: %w", spec.Name, err)
	}
	_ = writeLog(spec.LogPath, result.Stdout, result.Stderr)
	return RuntimeHandle{
		Backend:       "docker",
		ServiceName:   spec.Name,
		Identifier:    identifier(spec.ContainerName, spec.Name),
		ContainerName: spec.ContainerName,
		Command:       append([]string{"docker"}, args...),
		StartedAt:     backend.now(),
		LogPath:       spec.LogPath,
	}, nil
}

func (backend DockerBackend) StartRosbag(spec RosbagSpec) (RuntimeHandle, error) {
	service, err := RosbagServiceSpec(spec)
	if err != nil {
		return RuntimeHandle{}, err
	}
	return backend.StartService(service)
}

func (backend DockerBackend) RunProbe(spec ProbeSpec) (ProbeResult, error) {
	if err := spec.ValidateDocker(); err != nil {
		return ProbeResult{}, err
	}
	args, err := DockerProbeArgs(spec)
	if err != nil {
		return ProbeResult{}, err
	}
	ctx := context.Background()
	if spec.TimeoutSec > 0 {
		var cancel context.CancelFunc
		ctx, cancel = context.WithTimeout(ctx, time.Duration(spec.TimeoutSec*float64(time.Second)))
		defer cancel()
	}
	result, err := backend.runner().Run(ctx, "docker", args...)
	_ = writeLog(spec.LogPath, result.Stdout, result.Stderr)
	probe := ProbeResult{
		Backend:    "docker",
		Name:       spec.Name,
		ReturnCode: result.ExitCode,
		Stdout:     result.Stdout,
		Stderr:     result.Stderr,
		LogPath:    spec.LogPath,
	}
	if err != nil {
		return probe, fmt.Errorf("docker probe %s failed: %w", spec.Name, err)
	}
	return probe, nil
}

func (backend DockerBackend) Wait(handle RuntimeHandle) (int, error) {
	result, err := backend.runner().Run(context.Background(), "docker", "wait", handle.Identifier)
	if err != nil {
		return result.ExitCode, fmt.Errorf("docker wait %s failed: %w", handle.ServiceName, err)
	}
	if result.Stdout == "" {
		return 0, nil
	}
	code, parseErr := strconv.Atoi(firstLine(result.Stdout))
	if parseErr != nil {
		return 1, fmt.Errorf("docker wait %s returned non-integer status %q", handle.ServiceName, result.Stdout)
	}
	return code, nil
}

func (backend DockerBackend) Stop(handle RuntimeHandle) error {
	_, err := backend.runner().Run(context.Background(), "docker", "rm", "-f", handle.Identifier)
	if err != nil {
		return fmt.Errorf("docker stop %s failed: %w", handle.ServiceName, err)
	}
	return nil
}

func (backend DockerBackend) Logs(handle RuntimeHandle, tail int) (string, error) {
	if tail <= 0 {
		tail = 400
	}
	result, err := backend.runner().Run(context.Background(), "docker", "logs", "--tail", strconv.Itoa(tail), handle.Identifier)
	if err != nil {
		return result.Stdout, fmt.Errorf("docker logs %s failed: %w", handle.ServiceName, err)
	}
	return result.Stdout, nil
}

func DockerServiceArgs(spec ServiceSpec) ([]string, error) {
	if err := spec.ValidateDocker(); err != nil {
		return nil, err
	}
	args := []string{"run"}
	if spec.Detach {
		args = append(args, "--detach")
	}
	if spec.Remove {
		args = append(args, "--rm")
	}
	if spec.ContainerName != "" {
		args = append(args, "--name", spec.ContainerName)
	}
	for _, network := range spec.Networks {
		args = append(args, "--network", network)
	}
	for _, mount := range spec.Volumes {
		value, err := mount.DockerArg()
		if err != nil {
			return nil, err
		}
		args = append(args, "--volume", value)
	}
	if spec.CWD != "" {
		args = append(args, "--workdir", spec.CWD)
	}
	if spec.User != "" {
		args = append(args, "--user", spec.User)
	}
	for key, value := range spec.Env {
		args = append(args, "--env", key+"="+value)
	}
	args = append(args, spec.Image)
	args = append(args, spec.Command...)
	return args, nil
}

func DockerProbeArgs(spec ProbeSpec) ([]string, error) {
	if err := spec.ValidateDocker(); err != nil {
		return nil, err
	}
	service := ServiceSpec{
		Name:          spec.Name,
		Command:       spec.Command,
		Image:         spec.Image,
		ContainerName: spec.ContainerName,
		Env:           spec.Env,
		CWD:           spec.CWD,
		Volumes:       spec.Volumes,
		Networks:      spec.Networks,
		Detach:        false,
		Remove:        true,
		Required:      spec.Required,
		LogPath:       spec.LogPath,
		ServiceRole:   spec.ServiceRole,
	}
	return DockerServiceArgs(service)
}

func RosbagServiceSpec(spec RosbagSpec) (ServiceSpec, error) {
	topics, err := spec.Topics()
	if err != nil {
		return ServiceSpec{}, err
	}
	storage := spec.Storage
	if storage == "" {
		storage = "mcap"
	}
	command := []string{"ros2", "bag", "record", "-s", storage, "-o", spec.OutputPath, "--topics"}
	command = append(command, topics...)
	if spec.DurationSec > 0 {
		shellCommand := fmt.Sprintf("timeout --signal=INT %.1f %s", spec.DurationSec, shellJoin(command))
		command = []string{"bash", "-lc", shellCommand}
	}
	return ServiceSpec{
		Name:          spec.Name,
		Command:       command,
		Image:         spec.Image,
		ContainerName: spec.ContainerName,
		Env:           spec.Env,
		CWD:           spec.CWD,
		Volumes:       spec.Volumes,
		Networks:      spec.Networks,
		Detach:        true,
		Remove:        false,
		Required:      spec.Required,
		LogPath:       spec.LogPath,
		ServiceRole:   spec.ServiceRole,
	}, nil
}

func (backend DockerBackend) runner() CommandRunner {
	if backend.Runner == nil {
		return ExecRunner{}
	}
	return backend.Runner
}

func (backend DockerBackend) now() time.Time {
	if backend.Now != nil {
		return backend.Now()
	}
	return time.Now()
}

func identifier(containerName string, fallback string) string {
	if containerName != "" {
		return containerName
	}
	return fallback
}

func firstLine(value string) string {
	for index, char := range value {
		if char == '\n' || char == '\r' {
			return value[:index]
		}
	}
	return value
}

func shellJoin(args []string) string {
	quoted := make([]string, 0, len(args))
	for _, arg := range args {
		quoted = append(quoted, shellQuote(arg))
	}
	return joinWithSpace(quoted)
}

func shellQuote(value string) string {
	if value == "" {
		return "''"
	}
	return "'" + stringsReplaceAll(value, "'", "'\"'\"'") + "'"
}

func stringsReplaceAll(value string, old string, next string) string {
	if old == "" {
		return value
	}
	result := ""
	for {
		index := stringsIndex(value, old)
		if index < 0 {
			return result + value
		}
		result += value[:index] + next
		value = value[index+len(old):]
	}
}

func stringsIndex(value string, substr string) int {
	for i := 0; i+len(substr) <= len(value); i++ {
		if value[i:i+len(substr)] == substr {
			return i
		}
	}
	return -1
}

func joinWithSpace(values []string) string {
	if len(values) == 0 {
		return ""
	}
	result := values[0]
	for _, value := range values[1:] {
		result += " " + value
	}
	return result
}
