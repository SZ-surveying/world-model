from navlab.sim.companion.nodes.external_nav_source_selector import (
    SourceCandidate,
    select_external_nav_source,
)


def _scan_status(**overrides):
    data = {
        "ready": True,
        "quality_good": True,
        "uses_gazebo_truth_input": False,
        "uses_known_map_input": False,
        "valid_beams": 384,
        "inlier_ratio": 0.62,
        "residual_rms_m": 0.12,
        "correction_eligibility": {
            "correction_allowed": True,
            "allowed_axes": ["x", "y"],
            "direction_cosine_min": 0.99,
            "x_sign_flips": 0,
            "y_sign_flips": 0,
        },
    }
    data.update(overrides)
    return data


def test_stable_scan_reference_wins_over_near_zero_cartographer():
    decision = select_external_nav_source(
        slam=SourceCandidate(frame_id="map", child_frame_id="base_link", x_m=0.01, y_m=0.02),
        scan=SourceCandidate(frame_id="scan_reference", child_frame_id="base_link", x_m=0.27, y_m=-0.20),
        scan_status=_scan_status(),
        hover_phase="hover_hold",
        scan_status_age_ms=40.0,
    )

    assert decision.source == "scan_reference"
    assert decision.output_x_m == 0.27
    assert decision.output_y_m == -0.20
    assert decision.cartographer_scan_disagreement is True
    assert decision.uses_gazebo_truth_input is False
    assert decision.blockers == ()


def test_quality_bad_fails_closed_to_slam_passthrough():
    decision = select_external_nav_source(
        slam=SourceCandidate(frame_id="map", child_frame_id="base_link", x_m=0.01, y_m=0.02),
        scan=SourceCandidate(frame_id="scan_reference", child_frame_id="base_link", x_m=0.27, y_m=-0.20),
        scan_status=_scan_status(quality_good=False),
        hover_phase="hover_hold",
        scan_status_age_ms=40.0,
    )

    assert decision.source == "slam_passthrough"
    assert decision.output_x_m == 0.01
    assert decision.output_y_m == 0.02
    assert "scan_reference_quality_not_good" in decision.blockers


def test_stale_scan_status_fails_closed():
    decision = select_external_nav_source(
        slam=SourceCandidate(frame_id="map", child_frame_id="base_link", x_m=0.01, y_m=0.02),
        scan=SourceCandidate(frame_id="scan_reference", child_frame_id="base_link", x_m=0.27, y_m=-0.20),
        scan_status=_scan_status(),
        hover_phase="hover_hold",
        scan_status_age_ms=900.0,
    )

    assert decision.source == "slam_passthrough"
    assert "scan_reference_status_stale" in decision.blockers


def test_sign_flip_fails_closed():
    status = _scan_status(
        correction_eligibility={
            "correction_allowed": True,
            "allowed_axes": ["x", "y"],
            "direction_cosine_min": 0.99,
            "x_sign_flips": 1,
            "y_sign_flips": 0,
        }
    )
    decision = select_external_nav_source(
        slam=SourceCandidate(frame_id="map", child_frame_id="base_link", x_m=0.01, y_m=0.02),
        scan=SourceCandidate(frame_id="scan_reference", child_frame_id="base_link", x_m=0.27, y_m=-0.20),
        scan_status=status,
        hover_phase="hover_hold",
        scan_status_age_ms=40.0,
    )

    assert decision.source == "slam_passthrough"
    assert "scan_reference_x_sign_flips" in decision.blockers


def test_non_hover_phase_fails_closed():
    decision = select_external_nav_source(
        slam=SourceCandidate(frame_id="map", child_frame_id="base_link", x_m=0.01, y_m=0.02),
        scan=SourceCandidate(frame_id="scan_reference", child_frame_id="base_link", x_m=0.27, y_m=-0.20),
        scan_status=_scan_status(),
        hover_phase="complete",
        scan_status_age_ms=40.0,
    )

    assert decision.source == "slam_passthrough"
    assert "not_hover_correction_phase" in decision.blockers


def test_recent_good_scan_is_held_through_short_quality_drop():
    decision = select_external_nav_source(
        slam=SourceCandidate(frame_id="map", child_frame_id="base_link", x_m=0.01, y_m=0.02),
        scan=SourceCandidate(frame_id="scan_reference", child_frame_id="base_link", x_m=0.29, y_m=-0.22),
        scan_status=_scan_status(quality_good=False),
        hover_phase="hover_hold",
        scan_status_age_ms=40.0,
        last_good_scan=SourceCandidate(frame_id="scan_reference", child_frame_id="base_link", x_m=0.27, y_m=-0.20),
        last_good_scan_age_ms=120.0,
    )

    assert decision.source == "scan_reference_hold"
    assert decision.output_x_m == 0.27
    assert decision.output_y_m == -0.20
    assert decision.hold_age_ms == 120.0
    assert decision.hold_reason == "last_good_scan_reference_within_ttl"
    assert "scan_reference_quality_not_good" in decision.blockers


def test_recent_good_scan_hold_expires_after_ttl():
    decision = select_external_nav_source(
        slam=SourceCandidate(frame_id="map", child_frame_id="base_link", x_m=0.01, y_m=0.02),
        scan=SourceCandidate(frame_id="scan_reference", child_frame_id="base_link", x_m=0.29, y_m=-0.22),
        scan_status=_scan_status(quality_good=False),
        hover_phase="hover_hold",
        scan_status_age_ms=40.0,
        last_good_scan=SourceCandidate(frame_id="scan_reference", child_frame_id="base_link", x_m=0.27, y_m=-0.20),
        last_good_scan_age_ms=900.0,
        max_hold_age_ms=750.0,
    )

    assert decision.source == "slam_passthrough"
    assert decision.output_x_m == 0.01
    assert decision.output_y_m == 0.02


def test_recent_good_scan_hold_is_disabled_outside_hover_phase():
    decision = select_external_nav_source(
        slam=SourceCandidate(frame_id="map", child_frame_id="base_link", x_m=0.01, y_m=0.02),
        scan=SourceCandidate(frame_id="scan_reference", child_frame_id="base_link", x_m=0.29, y_m=-0.22),
        scan_status=_scan_status(quality_good=False),
        hover_phase="complete",
        scan_status_age_ms=40.0,
        last_good_scan=SourceCandidate(frame_id="scan_reference", child_frame_id="base_link", x_m=0.27, y_m=-0.20),
        last_good_scan_age_ms=120.0,
    )

    assert decision.source == "slam_passthrough"
    assert "not_hover_correction_phase" in decision.blockers
