from __future__ import annotations

from pathlib import Path


def _load_params() -> dict[str, str]:
    params: dict[str, str] = {}
    for raw_line in Path("profiles/navlab-sitl-external-nav.parm").read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        key, value = line.split(maxsplit=1)
        params[key] = value
    return params


def test_sitl_external_nav_matches_official_cartographer_sitl_baseline() -> None:
    params = _load_params()

    assert params["GPS_TYPE"] == "0"
    assert params["AHRS_EKF_TYPE"] == "3"
    assert params["EK2_ENABLE"] == "0"
    assert params["EK3_ENABLE"] == "1"
    assert params["VISO_TYPE"] == "1"
    assert params["ARMING_CHECK"] == "388598"
    assert params["EK3_SRC1_POSXY"] == "6"
    assert params["EK3_SRC1_POSZ"] == "1"
    assert params["EK3_SRC1_VELXY"] == "6"
    assert params["EK3_SRC1_VELZ"] == "6"
    assert params["EK3_SRC1_YAW"] == "6"


def test_down_rangefinder_remains_fcu_peripheral_not_primary_ekf_source() -> None:
    params = _load_params()

    assert params["RNGFND1_TYPE"] == "10"
    assert params["RNGFND1_ORIENT"] == "25"
    assert params["EK3_SRC1_POSZ"] == "1"
