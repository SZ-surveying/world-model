set dotenv-load := true
set dotenv-filename := ".env"

set shell := ["bash", "-euo", "pipefail", "-c"]

orchestration_cmd := "uv run --project orchestration python orchestration/main.py"

default:
    @just --list

# Check P8 official maze exploration gate prerequisites.
navlab-exploration-gate-doctor *args='':
    {{orchestration_cmd}} exploration-gate-doctor {{args}}

# Run P8 official maze exploration gate acceptance.
navlab-exploration-gate-acceptance duration_sec='150' *args='':
    {{orchestration_cmd}} exploration-gate-acceptance {{duration_sec}} {{args}}

# Run a representative P8 exploration replay for P9 Foxglove overlay.
navlab-exploration-replay duration_sec='180' profile='conservative' *args='':
    {{orchestration_cmd}} exploration-replay-acceptance {{duration_sec}} --profile {{profile}} {{args}}

# Run a longer P9 display replay for screenshots and Foxglove demos.
navlab-exploration-display-replay duration_sec='240' *args='':
    {{orchestration_cmd}} exploration-replay-acceptance {{duration_sec}} --profile display {{args}}

# Build a Foxglove-lite replay MCAP with the official maze overlay.
foxglove-replay date='':
    uv run --project orchestration python scripts/build_foxglove_replay_mcap.py {{date}}

# Dry-run the Foxglove-lite replay build for the latest run or a given run id.
foxglove-replay-dry-run date='':
    uv run --project orchestration python scripts/build_foxglove_replay_mcap.py {{date}} --dry-run

# Upload the latest raw/full P8 MCAP by default; pass --lite to upload/generate the lite MCAP.
foxglove-upload date='' *args='':
    scripts/upload_foxglove_mcap.py {{date}} --force {{args}}

# Check P10 body-fixed lidar scan integrity gate prerequisites.
navlab-scan-integrity-gate-doctor *args='':
    {{orchestration_cmd}} scan-integrity-gate-doctor {{args}}

# Run P10 body-fixed lidar scan integrity gate acceptance.
navlab-scan-integrity-gate-acceptance duration_sec='140' *args='':
    {{orchestration_cmd}} scan-integrity-gate-acceptance {{duration_sec}} {{args}}

# Check P11 bounded 2D lidar scan stabilization prerequisites.
navlab-scan-stabilization-gate-doctor *args='':
    {{orchestration_cmd}} scan-stabilization-gate-doctor {{args}}

# Run P11 bounded 2D lidar scan stabilization acceptance with P9 replay motion.
navlab-scan-stabilization-gate-acceptance duration_sec='240' *args='':
    {{orchestration_cmd}} scan-stabilization-gate-acceptance {{duration_sec}} {{args}}

# Check P12 airframe disturbance scan robustness gate prerequisites.
navlab-airframe-disturbance-gate-doctor *args='':
    {{orchestration_cmd}} airframe-disturbance-gate-doctor {{args}}

# Run P12 airframe disturbance profile-sweep acceptance.
navlab-airframe-disturbance-gate-acceptance duration_sec='240' *args='':
    {{orchestration_cmd}} airframe-disturbance-gate-acceptance {{duration_sec}} {{args}}
