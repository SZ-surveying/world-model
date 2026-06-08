set dotenv-load := true
set dotenv-filename := ".env"

set shell := ["bash", "-euo", "pipefail", "-c"]

orchestration_cmd := "uv run --project orchestration python orchestration/main.py"
command_cmd := "uv run --project scripts/command python scripts/command/main.py"

default:
    @just --list

# Run built-in hover acceptance.
navlab-hover duration_sec='90' *args='':
    {{orchestration_cmd}} hover {{duration_sec}} {{args}}

# Check built-in P8 movement/exploration prerequisites.
navlab-exploration-doctor *args='':
    {{orchestration_cmd}} exploration-doctor {{args}}

# Run built-in P8 movement/exploration acceptance.
navlab-exploration duration_sec='150' *args='':
    {{orchestration_cmd}} exploration {{duration_sec}} {{args}}

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

# Check P10 body-fixed lidar scan integrity gate prerequisites.
navlab-scan-integrity-gate-doctor *args='':
    {{orchestration_cmd}} scan-integrity-gate-doctor {{args}}

# Run P10 body-fixed lidar scan integrity gate acceptance.
navlab-scan-integrity-gate-acceptance duration_sec='140' *args='':
    {{orchestration_cmd}} scan-integrity-gate-acceptance {{duration_sec}} {{args}}
# Check built-in tilted-scan robustness prerequisites.
navlab-scan-robustness-doctor *args='':
    {{orchestration_cmd}} scan-robustness-doctor {{args}}

# Run built-in tilted-scan robustness acceptance.
navlab-scan-robustness duration_sec='240' *args='':
    {{orchestration_cmd}} scan-robustness {{duration_sec}} {{args}}

# Check process+real runtime preflight contract.
navlab-real-preflight-doctor *args='':
    {{orchestration_cmd}} real-preflight-doctor {{args}}
