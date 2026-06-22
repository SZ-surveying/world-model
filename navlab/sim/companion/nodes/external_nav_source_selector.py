from __future__ import annotations

import argparse
import copy
import json
import math
from dataclasses import dataclass
from typing import Any

CORRECTION_PHASES = frozenset({"hover_settle", "hover_hold"})
DEFAULT_SLAM_ODOM_TOPIC = "/slam/odom"
DEFAULT_SCAN_REFERENCE_ODOM_TOPIC = "/navlab/scan_reference_drift/odom"
DEFAULT_SCAN_REFERENCE_STATUS_TOPIC = "/navlab/scan_reference_drift/status"
DEFAULT_HOVER_STATUS_TOPIC = "/navlab/hover/status"
DEFAULT_OUTPUT_ODOM_TOPIC = "/external_nav/odom_candidate"
DEFAULT_STATUS_TOPIC = "/external_nav/source_selector/status"


@dataclass(frozen=True, slots=True)
class SourceCandidate:
    frame_id: str
    child_frame_id: str
    x_m: float
    y_m: float
    z_m: float = 0.0


@dataclass(frozen=True, slots=True)
class SourceDecision:
    source: str
    output_x_m: float
    output_y_m: float
    output_z_m: float
    output_frame_id: str
    output_child_frame_id: str
    blockers: tuple[str, ...]
    cartographer_scan_disagreement: bool
    uses_gazebo_truth_input: bool = False
    uses_known_map_input: bool = False
    hold_age_ms: float | None = None
    hold_reason: str | None = None


def _float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _bool(value: Any) -> bool:
    return bool(value) if isinstance(value, bool) else False


def _unique(values: list[str]) -> tuple[str, ...]:
    return tuple(dict.fromkeys(v for v in values if v))


def _magnitude(x: float, y: float) -> float:
    return math.hypot(x, y)


def _scan_is_eligible(
    status: dict[str, Any],
    *,
    hover_phase: str,
    scan_status_age_ms: float,
    max_status_age_ms: float,
    min_valid_beams: int,
    min_inlier_ratio: float,
    max_residual_rms_m: float,
    min_direction_cosine: float,
    max_axis_sign_flips: int,
) -> tuple[bool, tuple[str, ...]]:
    blockers: list[str] = []
    if hover_phase not in CORRECTION_PHASES:
        blockers.append("not_hover_correction_phase")
    if scan_status_age_ms < 0.0 or scan_status_age_ms > max_status_age_ms:
        blockers.append("scan_reference_status_stale")
    if _bool(status.get("uses_gazebo_truth_input")):
        blockers.append("scan_reference_uses_gazebo_truth")
    if _bool(status.get("uses_known_map_input")):
        blockers.append("scan_reference_uses_known_map")
    if not _bool(status.get("ready")):
        blockers.append("scan_reference_not_ready")
    if not _bool(status.get("quality_good")):
        blockers.append("scan_reference_quality_not_good")
    if _float(status.get("valid_beams")) < float(min_valid_beams):
        blockers.append("scan_reference_valid_beams_low")
    if _float(status.get("inlier_ratio")) < min_inlier_ratio:
        blockers.append("scan_reference_inlier_ratio_low")
    if _float(status.get("residual_rms_m")) > max_residual_rms_m:
        blockers.append("scan_reference_residual_high")

    eligibility = status.get("correction_eligibility") if isinstance(status.get("correction_eligibility"), dict) else {}
    if not _bool(eligibility.get("correction_allowed")):
        blockers.append("scan_reference_eligibility_not_allowed")
    allowed_axes = eligibility.get("allowed_axes") if isinstance(eligibility.get("allowed_axes"), list) else []
    if "x" not in allowed_axes or "y" not in allowed_axes:
        blockers.append("scan_reference_xy_axes_not_allowed")
    direction_cosine = eligibility.get("direction_cosine_min")
    if direction_cosine is not None and _float(direction_cosine, 1.0) < min_direction_cosine:
        blockers.append("scan_reference_direction_not_continuous")
    if int(_float(eligibility.get("x_sign_flips"))) > max_axis_sign_flips:
        blockers.append("scan_reference_x_sign_flips")
    if int(_float(eligibility.get("y_sign_flips"))) > max_axis_sign_flips:
        blockers.append("scan_reference_y_sign_flips")
    return not blockers, _unique(blockers)


def select_external_nav_source(
    *,
    slam: SourceCandidate,
    scan: SourceCandidate | None,
    scan_status: dict[str, Any],
    hover_phase: str,
    scan_status_age_ms: float,
    max_status_age_ms: float = 400.0,
    min_valid_beams: int = 80,
    min_inlier_ratio: float = 0.45,
    max_residual_rms_m: float = 0.30,
    min_direction_cosine: float = 0.70,
    max_axis_sign_flips: int = 0,
    cartographer_disagreement_m: float = 0.15,
    last_good_scan: SourceCandidate | None = None,
    last_good_scan_age_ms: float = -1.0,
    max_hold_age_ms: float = 750.0,
    max_hold_jump_m: float = 1.25,
) -> SourceDecision:
    if scan is None:
        return SourceDecision(
            source="slam_passthrough",
            output_x_m=slam.x_m,
            output_y_m=slam.y_m,
            output_z_m=slam.z_m,
            output_frame_id=slam.frame_id,
            output_child_frame_id=slam.child_frame_id,
            blockers=("scan_reference_odom_missing",),
            cartographer_scan_disagreement=False,
        )

    eligible, blockers = _scan_is_eligible(
        scan_status,
        hover_phase=hover_phase,
        scan_status_age_ms=scan_status_age_ms,
        max_status_age_ms=max_status_age_ms,
        min_valid_beams=min_valid_beams,
        min_inlier_ratio=min_inlier_ratio,
        max_residual_rms_m=max_residual_rms_m,
        min_direction_cosine=min_direction_cosine,
        max_axis_sign_flips=max_axis_sign_flips,
    )
    disagreement = _magnitude(scan.x_m - slam.x_m, scan.y_m - slam.y_m) > cartographer_disagreement_m
    if not eligible:
        hold_allowed = (
            hover_phase in CORRECTION_PHASES
            and last_good_scan is not None
            and 0.0 <= last_good_scan_age_ms <= max_hold_age_ms
            and 0.0 <= scan_status_age_ms <= max_status_age_ms
            and not _bool(scan_status.get("uses_gazebo_truth_input"))
            and not _bool(scan_status.get("uses_known_map_input"))
            and _magnitude(last_good_scan.x_m - slam.x_m, last_good_scan.y_m - slam.y_m) <= max_hold_jump_m
        )
        if hold_allowed:
            return SourceDecision(
                source="scan_reference_hold",
                output_x_m=last_good_scan.x_m,
                output_y_m=last_good_scan.y_m,
                output_z_m=slam.z_m,
                output_frame_id=slam.frame_id,
                output_child_frame_id=slam.child_frame_id,
                blockers=blockers,
                cartographer_scan_disagreement=disagreement,
                hold_age_ms=last_good_scan_age_ms,
                hold_reason="last_good_scan_reference_within_ttl",
            )
        return SourceDecision(
            source="slam_passthrough",
            output_x_m=slam.x_m,
            output_y_m=slam.y_m,
            output_z_m=slam.z_m,
            output_frame_id=slam.frame_id,
            output_child_frame_id=slam.child_frame_id,
            blockers=blockers,
            cartographer_scan_disagreement=disagreement,
        )

    return SourceDecision(
        source="scan_reference",
        output_x_m=scan.x_m,
        output_y_m=scan.y_m,
        output_z_m=slam.z_m,
        output_frame_id=slam.frame_id,
        output_child_frame_id=slam.child_frame_id,
        blockers=(),
        cartographer_scan_disagreement=disagreement,
    )


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Select a fail-closed ExternalNav odometry candidate for hover.")
    parser.add_argument("--slam-odom-topic", default=DEFAULT_SLAM_ODOM_TOPIC)
    parser.add_argument("--scan-reference-odom-topic", default=DEFAULT_SCAN_REFERENCE_ODOM_TOPIC)
    parser.add_argument("--scan-reference-status-topic", default=DEFAULT_SCAN_REFERENCE_STATUS_TOPIC)
    parser.add_argument("--hover-status-topic", default=DEFAULT_HOVER_STATUS_TOPIC)
    parser.add_argument("--output-odom-topic", default=DEFAULT_OUTPUT_ODOM_TOPIC)
    parser.add_argument("--status-topic", default=DEFAULT_STATUS_TOPIC)
    parser.add_argument("--max-status-age-ms", type=float, default=400.0)
    parser.add_argument("--max-hold-age-ms", type=float, default=750.0)
    parser.add_argument("--max-hold-jump-m", type=float, default=1.25)
    parser.add_argument("--cartographer-disagreement-m", type=float, default=0.15)
    return parser


def _parse_json(data: str) -> dict[str, Any]:
    try:
        parsed = json.loads(data)
    except json.JSONDecodeError:
        return {}
    return parsed if isinstance(parsed, dict) else {}


def _phase_from_status(data: str) -> str:
    return str(_parse_json(data).get("phase") or "")


def run(argv: list[str] | None = None) -> int:
    args = _build_arg_parser().parse_args(argv)
    try:
        import rclpy
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
        from std_msgs.msg import String
    except ModuleNotFoundError as exc:
        raise SystemExit("external_nav_source_selector requires ROS2 Python packages.") from exc

    class ExternalNavSourceSelectorNode(Node):
        def __init__(self) -> None:
            super().__init__("navlab_external_nav_source_selector")
            self._hover_phase = ""
            self._last_scan_odom: Odometry | None = None
            self._last_scan_status: dict[str, Any] = {}
            self._last_scan_status_wall = 0.0
            self._last_good_scan_candidate: SourceCandidate | None = None
            self._last_good_scan_wall = 0.0
            self._last_decision = SourceDecision(
                source="waiting",
                output_x_m=0.0,
                output_y_m=0.0,
                output_z_m=0.0,
                output_frame_id="map",
                output_child_frame_id="base_link",
                blockers=("waiting_for_slam_odom",),
                cartographer_scan_disagreement=False,
            )
            output_qos = QoSProfile(depth=10)
            output_qos.reliability = ReliabilityPolicy.RELIABLE
            self._odom_pub = self.create_publisher(Odometry, args.output_odom_topic, output_qos)
            self._status_pub = self.create_publisher(String, args.status_topic, 10)
            self.create_subscription(Odometry, args.slam_odom_topic, self._handle_slam_odom, qos_profile_sensor_data)
            self.create_subscription(
                Odometry,
                args.scan_reference_odom_topic,
                self._handle_scan_odom,
                qos_profile_sensor_data,
            )
            self.create_subscription(String, args.scan_reference_status_topic, self._handle_scan_status, 10)
            self.create_subscription(String, args.hover_status_topic, self._handle_hover_status, 10)
            self.create_timer(0.5, self._publish_status)

        def _handle_hover_status(self, message: String) -> None:
            phase = _phase_from_status(message.data)
            if phase:
                self._hover_phase = phase

        def _handle_scan_status(self, message: String) -> None:
            self._last_scan_status = _parse_json(message.data)
            self._last_scan_status_wall = self.get_clock().now().nanoseconds / 1e9

        def _handle_scan_odom(self, message: Odometry) -> None:
            self._last_scan_odom = message

        def _candidate_from_odom(self, message: Odometry) -> SourceCandidate:
            return SourceCandidate(
                frame_id=message.header.frame_id,
                child_frame_id=message.child_frame_id,
                x_m=float(message.pose.pose.position.x),
                y_m=float(message.pose.pose.position.y),
                z_m=float(message.pose.pose.position.z),
            )

        def _handle_slam_odom(self, message: Odometry) -> None:
            now_sec = self.get_clock().now().nanoseconds / 1e9
            status_age_ms = (
                -1.0 if self._last_scan_status_wall <= 0.0 else (now_sec - self._last_scan_status_wall) * 1000.0
            )
            scan_candidate = (
                self._candidate_from_odom(self._last_scan_odom) if self._last_scan_odom is not None else None
            )
            slam_candidate = self._candidate_from_odom(message)
            last_good_age_ms = (
                -1.0 if self._last_good_scan_wall <= 0.0 else (now_sec - self._last_good_scan_wall) * 1000.0
            )
            decision = select_external_nav_source(
                slam=slam_candidate,
                scan=scan_candidate,
                scan_status=self._last_scan_status,
                hover_phase=self._hover_phase,
                scan_status_age_ms=status_age_ms,
                max_status_age_ms=args.max_status_age_ms,
                cartographer_disagreement_m=args.cartographer_disagreement_m,
                last_good_scan=self._last_good_scan_candidate,
                last_good_scan_age_ms=last_good_age_ms,
                max_hold_age_ms=args.max_hold_age_ms,
                max_hold_jump_m=args.max_hold_jump_m,
            )
            if decision.source == "scan_reference" and scan_candidate is not None:
                self._last_good_scan_candidate = scan_candidate
                self._last_good_scan_wall = now_sec
            out = copy.deepcopy(message)
            out.header.frame_id = decision.output_frame_id
            out.child_frame_id = decision.output_child_frame_id
            out.pose.pose.position.x = decision.output_x_m
            out.pose.pose.position.y = decision.output_y_m
            out.pose.pose.position.z = decision.output_z_m
            out.pose.covariance[0] = max(float(out.pose.covariance[0]), 0.04)
            out.pose.covariance[7] = max(float(out.pose.covariance[7]), 0.04)
            self._odom_pub.publish(out)
            self._last_decision = decision

        def _publish_status(self) -> None:
            payload = {
                "ready": self._last_decision.source not in {"waiting"},
                "source": self._last_decision.source,
                "blockers": list(self._last_decision.blockers),
                "hover_phase": self._hover_phase,
                "output_odom_topic": args.output_odom_topic,
                "slam_odom_topic": args.slam_odom_topic,
                "scan_reference_odom_topic": args.scan_reference_odom_topic,
                "scan_reference_status_topic": args.scan_reference_status_topic,
                "cartographer_scan_disagreement": self._last_decision.cartographer_scan_disagreement,
                "uses_gazebo_truth_input": self._last_decision.uses_gazebo_truth_input,
                "uses_known_map_input": self._last_decision.uses_known_map_input,
                "output_frame_id": self._last_decision.output_frame_id,
                "output_child_frame_id": self._last_decision.output_child_frame_id,
                "hold_age_ms": self._last_decision.hold_age_ms,
                "hold_reason": self._last_decision.hold_reason,
            }
            msg = String()
            msg.data = json.dumps(payload, sort_keys=True)
            self._status_pub.publish(msg)

    rclpy.init(args=None)
    node = ExternalNavSourceSelectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(run())
