# Stage 1 Height Estimate Contract

## Decision

Stage 1 does not require an independent height sensor to pass. The accepted SITL path still uses ExternalNav odometry and ArduPilot local setpoints.

This contract only reserves the height input so Stage 2 can add barometer, rangefinder, depth, or LiDAR-derived height without changing the SLAM and ExternalNav topic chain.

## Topic

```text
/height/estimate
```

Type:

```text
std_msgs/msg/String
```

Payload is compact JSON:

```json
{
  "z": 0.82,
  "vz": 0.01,
  "covariance": 0.04,
  "source_type": "rangefinder"
}
```

## Field Semantics

| Field | Required | Unit | Meaning |
| --- | --- | --- | --- |
| `z` | yes | m | Height estimate in the local vertical axis, positive upward from the local reference plane |
| `vz` | yes | m/s | Vertical velocity, positive upward |
| `covariance` | yes | m^2 | Scalar variance for `z`; lower is better |
| `source_type` | yes | string | Producer class such as `barometer`, `rangefinder`, `lidar_floor`, `depth`, `motion_capture`, or `synthetic` |

## Bridge Behavior

`external_nav_bridge` subscribes to `/height/estimate` and exposes height health inside `/external_nav/status`.

Relevant parameters:

```yaml
height_topic: /height/estimate
height_timeout_ms: 500
require_height_for_output: false
max_height_covariance: 4.0
```

Default Stage 1 behavior:

- `require_height_for_output=false`
- `/height/estimate` may be absent
- ExternalNav output can still be healthy when odom and required IMU inputs are healthy
- `/external_nav/status.height` reports whether the height input is present, fresh, parseable, and covariance-valid

If `require_height_for_output=true`, bridge output is gated until:

- a `/height/estimate` message has arrived
- the message age is less than `height_timeout_ms`
- all required JSON fields parse correctly
- `covariance >= 0`
- `covariance <= max_height_covariance`

## Status Shape

`/external_nav/status` includes:

```json
{
  "height": {
    "required": false,
    "present": true,
    "fresh": true,
    "parse_ok": true,
    "covariance_ok": true,
    "age_ms": 12.3,
    "topic": "/height/estimate",
    "max_covariance": 4.0,
    "source_type": "rangefinder",
    "z": 0.82,
    "vz": 0.01,
    "covariance": 0.04
  }
}
```

## Boundaries

- Stage 1 does not fuse `/height/estimate` into `/external_nav/odom`.
- Stage 1 does not send height-specific MAVLink messages.
- The height topic is a gating and diagnostics input only until a real altitude fusion policy is designed.
- Future producers should publish the same contract regardless of whether the source is simulated or real hardware.
