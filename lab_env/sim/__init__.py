"""Simulation helpers for Gazebo-backed scan workflows."""

from lab_env.sim.perception.contract import DEFAULT_SCAN_CONTRACT, ScanContract
from lab_env.sim.perception.front_sector import (
    ForwardStopDecision,
    ForwardStopStateMachine,
    FrontSectorReport,
    classify_front_min,
    classify_front_sector,
    compute_front_min,
)
from lab_env.sim.perception.scan_features import ScanFeaturesReport, compute_scan_features
from lab_env.sim.status import DEFAULT_SIM_LOG_TOPIC, encode_sim_log
from lab_env.sim.waypoints import StraightLineMission, Waypoint, load_straight_line_mission
from lab_env.sim.world.world_markers import (
    MarkerColor,
    MarkerPose,
    MarkerScale,
    MarkerSpec,
    WorldObstacleBox,
    load_world_marker_specs,
    load_world_obstacle_boxes,
    synthesize_planar_scan,
)

__all__ = [
    "DEFAULT_SCAN_CONTRACT",
    "DEFAULT_SIM_LOG_TOPIC",
    "ForwardStopDecision",
    "ForwardStopStateMachine",
    "FrontSectorReport",
    "MarkerColor",
    "MarkerPose",
    "MarkerScale",
    "MarkerSpec",
    "ScanContract",
    "ScanFeaturesReport",
    "StraightLineMission",
    "Waypoint",
    "WorldObstacleBox",
    "classify_front_min",
    "classify_front_sector",
    "compute_front_min",
    "compute_scan_features",
    "encode_sim_log",
    "load_straight_line_mission",
    "load_world_marker_specs",
    "load_world_obstacle_boxes",
    "synthesize_planar_scan",
]
