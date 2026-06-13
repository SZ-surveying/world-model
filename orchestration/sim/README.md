# NavLab Simulation Orchestration

`orchestration/sim` is the Go control plane for simulation tasks. It owns the
simulation task registry, YAML task config loading, Docker runtime planning,
artifact writing, generated contract emission, image build orchestration, and
terminal replay UI.

## Scope

This package owns:

- Go CLI entrypoint `navlab-sim`.
- Project config loading from `config.toml` with Viper-compatible TOML shape.
- Task config loading from `configs/tasks/*.yaml`.
- Built-in simulation task registry for `hover`, `exploration`, and
  `scan-robustness`.
- Docker runtime service/probe plans.
- Runtime artifact generation and task result summaries.
- TUI replay for simulation artifact directories.
- Docker image build command parity for base, infra, and runtime images.

This package does not own:

- Python runtime node implementations. See [`../../navlab/README.md`](../../navlab/README.md).
- Dockerfile content and profile assets. See [`../../docker/README.md`](../../docker/README.md).
- Real-machine task execution. See [`../real/README.md`](../real/README.md).
- Protobuf schema definitions. See [`../../contracts/README.md`](../../contracts/README.md).

## Layout

```text
orchestration/sim/
  cmd/navlab-sim/
  configs/tasks/
  internal/
    artifacts/
    config/
    images/
    runtime/
    tasks/
    tui/
    ui/
  config.toml
  go.mod
```

- `cmd/navlab-sim/` contains Cobra CLI wiring.
- `internal/config/` loads project TOML and task YAML.
- `internal/tasks/` owns task registry, planning, runtime specs, blocker
  evaluation, and summary generation.
- `internal/runtime/` owns Docker runtime command construction.
- `internal/images/` owns Docker image build resolution.
- `internal/artifacts/` writes dry-run and runtime artifacts.
- `internal/tui/` owns replay UI and live event presentation.

## Commands

Run from `orchestration/sim`.

```bash
go run ./cmd/navlab-sim doctor
go run ./cmd/navlab-sim list-tasks
go run ./cmd/navlab-sim show-task hover
```

Plan a task without starting Docker services:

```bash
go run ./cmd/navlab-sim run hover --dry-run
go run ./cmd/navlab-sim run exploration --dry-run
go run ./cmd/navlab-sim run scan-robustness --dry-run
```

Open a replay TUI for a generated artifact directory:

```bash
go run ./cmd/navlab-sim tui ../../artifacts/sim/RUN_ID
```

Build simulation images through the Go entrypoint:

```bash
go run ./cmd/navlab-sim build base
go run ./cmd/navlab-sim build infra
go run ./cmd/navlab-sim build runtime
go run ./cmd/navlab-sim build all
```

See [`../../docker/README.md`](../../docker/README.md) for Docker image layout,
tag policy, distro selection, and single-image build examples.

## Configuration

Project-level configuration lives in `config.toml`.

Important sections:

- `[orchestration]`: must identify `family = "sim"` and
  `implementation = "go"`.
- `[orchestration.runtime]`: must use simulation mode and Docker backend.
- `[paths]`: workspace root, artifact root, and task config directory.
- `[navlab.images]`: image tag policy and image definitions.
- task-specific runtime sections such as `[landing]`, `[sitl]`, `[slam]`, and
  `[official_baseline]`.

Task configs live under `configs/tasks/*.yaml`. Each task config declares:

- `id`
- `family`
- `description`
- `capabilities`
- `task` defaults
- task-specific parameter blocks

Current built-in tasks:

- `hover`
- `exploration`
- `scan-robustness`

## Contracts And Artifacts

The sim orchestrator writes artifacts under the configured sim artifact root,
currently `../../artifacts/sim`. Generated files include runtime plans, task
requests/results, summaries, and replay inputs.

Cross-language outputs should use generated Go protobuf types from
`navlab/contracts/gen/go` where possible. Contract definitions and regeneration
commands are documented in [`../../contracts/README.md`](../../contracts/README.md).

## Development Commands

Format and lint affected Go files:

```bash
../../scripts/quality/format-go.sh --scope modified
```

Run Go tests:

```bash
go test ./...
```

Run the Go quality profile from the repository root:

```bash
./scripts/quality/check-go.sh
```

Verify a Docker image build command without building:

```bash
go run ./cmd/navlab-sim build infra --image gazebo-headless --distro jazzy --tag local --dry-run
```

## Maintenance Rules

- Keep task registration explicit in the Go task registry.
- Keep task configs in YAML under `configs/tasks`.
- Keep project-level config in `config.toml`.
- Keep Dockerfile ownership in `docker/images`, not under this package.
- Do not add real-machine behavior or hardware safety policy here.
- Add tests when changing config loading, task registry behavior, image build
  resolution, runtime specs, artifact schema, or TUI event parsing.
