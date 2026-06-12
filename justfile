set dotenv-load := true
set dotenv-filename := ".env"

set shell := ["bash", "-euo", "pipefail", "-c"]

sim_cmd := "cd orchestration/sim && go run ./cmd/navlab-sim"
real_cmd := "cd orchestration/real && cargo run --quiet --"
command_cmd := "uv run --project scripts/command python scripts/command/main.py"

default:
    @just --list

# Orchestration tasks

# Build orchestration runtime images.
navlab-build kind='all' *args='':
    {{sim_cmd}} build {{kind}} {{args}}

# Check Go sim orchestration.
check-go *args='':
    ./scripts/quality/check-go.sh {{args}}

# Format Go sim orchestration.
format-go *args='':
    ./scripts/quality/format-go.sh {{args}}

# Check Rust real orchestration.
check-rust *args='':
    ./scripts/quality/check-rust.sh {{args}}

# Format Rust real orchestration.
format-rust *args='':
    ./scripts/quality/format-rust.sh {{args}}

# Check Python projects.
check-python *args='':
    ./scripts/quality/check-python.sh {{args}}

# Format Python projects.
format-python *args='':
    ./scripts/quality/format-python.sh {{args}}

# Check sim orchestration.
navlab-doctor *args='':
    {{sim_cmd}} doctor {{args}}

# Check real orchestration.
navlab-real-doctor *args='':
    {{real_cmd}} doctor {{args}}

# Run a simulation task through Go orchestration.
navlab-run task *args='':
    {{sim_cmd}} run {{task}} {{args}}

# Run a real task through Rust orchestration.
navlab-real-run task *args='':
    {{real_cmd}} run {{task}} {{args}}

# Command tools

# Build a Foxglove-lite replay MCAP with the official maze overlay.
foxglove-replay date='':
    {{command_cmd}} foxglove build-replay {{date}}

# Upload the latest raw/full P8 MCAP by default; pass --lite to upload/generate the lite MCAP.
foxglove-upload date='' *args='':
    {{command_cmd}} foxglove upload {{date}} --force {{args}}

# Run the serial bridge from the scripts/command Python 3.11 project.
serial-bridge *args='':
    {{command_cmd}} serial bridge {{args}}
