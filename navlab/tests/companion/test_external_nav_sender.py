from __future__ import annotations

import math
import time
from types import SimpleNamespace

from navlab.common.pose import quaternion_from_yaw
from navlab.real.companion.nodes.external_nav import MavlinkExternalNavSender, _yaw_from_ros_quat_enu


def _pose_with_yaw(yaw_rad: float) -> SimpleNamespace:
    qx, qy, qz, qw = quaternion_from_yaw(yaw_rad)
    return SimpleNamespace(orientation=SimpleNamespace(x=qx, y=qy, z=qz, w=qw))


def _yaw_from_frd_quat(q: list[float]) -> float:
    w, x, y, z = q
    return math.atan2(2.0 * ((w * z) + (x * y)), 1.0 - (2.0 * ((y * y) + (z * z))))


def test_odometry_quaternion_aligns_initial_slam_yaw_to_fcu_attitude() -> None:
    sender = MavlinkExternalNavSender.__new__(MavlinkExternalNavSender)
    sender._use_fcu_roll_pitch = True
    sender._align_yaw_to_fcu = True
    sender._fcu_roll_rad = 0.0
    sender._fcu_pitch_rad = 0.0
    sender._fcu_yaw_rad = -0.5
    sender._last_fcu_attitude_monotonic = time.monotonic()
    sender._yaw_alignment_offset_rad = None

    first = sender._odometry_quaternion(_pose_with_yaw(0.1))
    second = sender._odometry_quaternion(_pose_with_yaw(0.2))

    assert math.isclose(_yaw_from_frd_quat(first), -0.5, abs_tol=1e-6)
    assert sender._yaw_alignment_offset_rad is not None
    assert math.isclose(_yaw_from_frd_quat(second), -0.6, abs_tol=1e-6)


def test_odometry_quaternion_can_send_raw_slam_yaw_without_alignment() -> None:
    sender = MavlinkExternalNavSender.__new__(MavlinkExternalNavSender)
    sender._use_fcu_roll_pitch = True
    sender._align_yaw_to_fcu = False
    sender._fcu_roll_rad = 0.0
    sender._fcu_pitch_rad = 0.0
    sender._fcu_yaw_rad = -0.5
    sender._last_fcu_attitude_monotonic = time.monotonic()
    sender._yaw_alignment_offset_rad = None

    q = sender._odometry_quaternion(_pose_with_yaw(0.1))

    assert math.isclose(_yaw_from_frd_quat(q), -0.1, abs_tol=1e-6)
    assert sender._yaw_alignment_offset_rad is None
    assert math.isclose(_yaw_from_ros_quat_enu(_pose_with_yaw(0.1).orientation), 0.1, abs_tol=1e-6)
