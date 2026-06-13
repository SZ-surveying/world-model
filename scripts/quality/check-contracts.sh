#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PROTO_DIR="$ROOT_DIR/contracts/proto"
EXAMPLES_DIR="$ROOT_DIR/contracts/examples"

log_step() {
  printf '\n[contracts] %s\n' "$1"
}

fail() {
  printf '[contracts] %s\n' "$1" >&2
  exit 1
}

command -v jq >/dev/null 2>&1 || fail "jq is required"
command -v protoc >/dev/null 2>&1 || fail "protoc is required"

log_step "Validating golden JSON examples"
find "$EXAMPLES_DIR" -name '*.json' -print0 | sort -z | xargs -0 -n1 jq empty

require_field() {
  local file="$1"
  local field="$2"

  jq -e --arg field "$field" 'has($field)' "$file" >/dev/null ||
    fail "$file is missing required field $field"
}

log_step "Validating required schemaVersion fields"
require_field "$EXAMPLES_DIR/orchestration/sim_task_request.json" "schemaVersion"
require_field "$EXAMPLES_DIR/orchestration/real_task_result.json" "schemaVersion"
require_field "$EXAMPLES_DIR/orchestration/doctor_result_blocked.json" "schemaVersion"
require_field "$EXAMPLES_DIR/runtime/sim_runtime_plan.json" "schemaVersion"
require_field "$EXAMPLES_DIR/runtime/real_process_event.json" "schemaVersion"

log_step "Validating proto syntax"
mapfile -t PROTO_FILES < <(find "$PROTO_DIR" -name '*.proto' | sort)
if [ "${#PROTO_FILES[@]}" -eq 0 ]; then
  fail "no proto files found under $PROTO_DIR"
fi
protoc -I "$PROTO_DIR" --include_imports --descriptor_set_out=/tmp/navlab_contracts.pb "${PROTO_FILES[@]}"

log_step "Contracts checks passed"
