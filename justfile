set dotenv-load := true
set dotenv-filename := ".env"

set shell := ["bash", "-euo", "pipefail", "-c"]

orchestration_cmd := "uv run --project orchestration python orchestration/main.py"
command_cmd := "uv run --project scripts/command python scripts/command/main.py"

default:
    @just --list

# Orchestration tasks

# Build orchestration runtime images.
navlab-build kind='all' *args='':
    {{orchestration_cmd}} build {{kind}} {{args}}

# Check the active runtime. Set NAVLAB_RUNTIME_BACKEND/MODE for real preflight.
navlab-doctor *args='':
    {{orchestration_cmd}} doctor {{args}}

# Run a built-in task through the unified wrapper: hover, exploration, or scan-robustness.
navlab-run task duration_sec='' *args='':
    if [ -n "{{duration_sec}}" ]; then \
        {{orchestration_cmd}} run {{task}} --duration-sec {{duration_sec}} {{args}}; \
    else \
        {{orchestration_cmd}} run {{task}} {{args}}; \
    fi

# Command tools

# Build a Foxglove-lite replay MCAP with the official maze overlay.
foxglove-replay date='':
    {{command_cmd}} foxglove build-replay {{date}}

# Dry-run the Foxglove-lite replay build for the latest run or a given run id.
foxglove-replay-dry-run date='':
    {{command_cmd}} foxglove build-replay {{date}} --dry-run

# Upload the latest raw/full P8 MCAP by default; pass --lite to upload/generate the lite MCAP.
foxglove-upload date='' *args='':
    {{command_cmd}} foxglove upload {{date}} --force {{args}}

# Run the serial bridge from the scripts/command Python 3.11 project.
serial-bridge *args='':
    {{command_cmd}} serial bridge {{args}}
