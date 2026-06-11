# Project Map

| Subsystem | Purpose | Owning files | Related tests | Confidence | Notes |
| --- | --- | --- | --- | --- | --- |
| Real run wrapper | Dispatches built-in real/simulation tasks and enforces operator safety confirmations. | `orchestration/src/cli.py` | `orchestration/tests/test_cli.py` | confirmed | `motor-debug` is process+real only and now exposes the Guided-mode prerequisite in dry-run output. |
| FCU controller helper | Generates the ROS/MAVLink FCU controller runtime script for GUIDED, arm, takeoff, motion intents, and landing. | `orchestration/src/tasks/helpers/fcu.py` | `orchestration/tests/test_config.py` | confirmed | Both official DDS and MAVLink bootstrap routes switch to Guided before arm/takeoff. |
| Companion hover mission | Runs direct MAVLink hover/landing mission logic from the companion image. | `navlab/companion/nodes/hover_mission.py` | `navlab/tests/companion/test_hover_mission.py` | confirmed | Defaults to `GUIDED`; waits for expected mode before arm/takeoff and sends Guided local-position setpoints after airborne. |
| Companion obstacle mission | Runs direct MAVLink obstacle/avoidance mission logic. | `navlab/companion/nodes/obstacle_mission.py` | `navlab/tests/companion/test_obstacle_mission.py` | confirmed | Defaults to `GUIDED`; local NED position/yaw targets are sent only after airborne. |
| Real motor debug | Performs no-props sequential motor tests for hardware debugging. | `orchestration/src/tasks/motor_debug.py` | `orchestration/tests/test_motor_debug.py`, `orchestration/tests/test_cli.py` | confirmed | Must switch to and observe `GUIDED` before sending `MAV_CMD_DO_MOTOR_TEST`. |
