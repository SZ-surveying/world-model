# NavLab Contracts

This package owns language-neutral contracts shared by `navlab` runtimes and
`orchestration`.

`orchestration` chooses artifact paths and starts processes. Runtime tasks write
their result artifacts to those paths. The contract here defines the stable
shape those artifacts must follow so future readers can be implemented in
Python, Rust, or another language without importing runtime code.

Current scope:

- `proto/navlab/runtime/v1/task_result.proto`: common runtime task result schema.

Current implementation note:

- Existing tasks still write `summary.json`.
- New or migrated task summaries should include
  `schema_version = "navlab.runtime.task_result.v1"` and stay mappable to the
  proto schema.
- Protobuf code generation is intentionally not wired in yet.
