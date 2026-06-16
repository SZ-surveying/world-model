from __future__ import annotations

from navlab.real.companion.nodes.height_estimator import parse_args


def test_height_estimator_defaults_publish_external_nav_height_contract() -> None:
    args = parse_args([])

    assert args.range_topic == "/rangefinder/down/range"
    assert args.height_topic == "/height/estimate"
    assert args.status_topic == "/height/status"
    assert args.source_type == "rangefinder_down_relative"
    assert args.covariance > 0
