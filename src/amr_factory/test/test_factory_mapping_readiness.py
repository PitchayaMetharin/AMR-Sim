import importlib.util
import math
import sys
from pathlib import Path
from unittest.mock import patch

from nav_msgs.msg import OccupancyGrid
import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
READINESS_PATH = PACKAGE_ROOT / "scripts" / "factory_mapping_readiness.py"
READINESS_SPEC = importlib.util.spec_from_file_location(
    "factory_mapping_readiness_contract", READINESS_PATH)
READINESS = importlib.util.module_from_spec(READINESS_SPEC)
sys.modules[READINESS_SPEC.name] = READINESS
READINESS_SPEC.loader.exec_module(READINESS)


def _valid_map():
    message = OccupancyGrid()
    message.header.frame_id = "map"
    message.info.width = 2
    message.info.height = 2
    message.info.resolution = 0.05
    message.info.origin.orientation.w = 1.0
    message.data = [-1, 0, -1, -1]
    return message


def test_map_gate_requires_valid_dimensions_and_known_free_cell():
    assert READINESS.valid_factory_map(_valid_map())

    malformed = _valid_map()
    malformed.data = [-1, 0]
    assert not READINESS.valid_factory_map(malformed)

    unknown = _valid_map()
    unknown.data = [-1, -1, -1, -1]
    assert not READINESS.valid_factory_map(unknown)

    nonfinite = _valid_map()
    nonfinite.info.resolution = math.nan
    assert not READINESS.valid_factory_map(nonfinite)


def test_map_gate_requires_exact_map_frame():
    message = _valid_map()
    message.header.frame_id = "odom"
    assert not READINESS.valid_factory_map(message)


def test_receipt_freshness_uses_bounded_steady_age():
    with patch.object(READINESS.time, "monotonic", return_value=10.0):
        assert READINESS.fresh_receipt(9.5, 1.0)
        assert not READINESS.fresh_receipt(8.9, 1.0)
        assert not READINESS.fresh_receipt(10.1, 1.0)


def test_composed_map_base_tf_requires_both_fresh_edges():
    gate = READINESS.MappingReadiness.__new__(READINESS.MappingReadiness)
    gate._tf_receipt_at = {
        ("map", "odom"): 10.0,
        ("odom", "base_footprint"): 10.0,
    }
    assert gate._fresh_tf("map", "base_footprint", now=10.5)
    gate._tf_receipt_at[("map", "odom")] = 8.9
    assert not gate._fresh_tf("map", "base_footprint", now=10.5)


def _adapter_gate(scan_at, tf_at, all_active=True):
    gate = READINESS.MappingReadiness.__new__(READINESS.MappingReadiness)
    gate._stage = "adapters"
    gate._last_scan_at = scan_at
    gate._tf_receipt_at = {("odom", "base_footprint"): tf_at} if tf_at is not None else {}
    gate._all_active = lambda _deadline: all_active
    return gate


def test_adapters_gate_rechecks_steady_time_after_lifecycle_callbacks():
    gate = _adapter_gate(0.0, 0.0)

    def lifecycle_checks(_deadline):
        # Model a lifecycle service spin advancing the steady clock before
        # delivering the newer scan/TF callbacks.
        READINESS.time.monotonic()
        gate._last_scan_at = 10.25
        gate._tf_receipt_at[("odom", "base_footprint")] = 10.25
        return True

    gate._all_active = lifecycle_checks
    with patch.object(READINESS.time, "monotonic", side_effect=[10.0, 10.25]):
        assert gate._stage_ready(deadline=20.0)


@pytest.mark.parametrize(
    ("scan_at", "tf_at"),
    [(8.9, 10.0), (10.0, 8.9), (None, 10.0), (10.0, None)],
)
def test_adapters_gate_rejects_stale_or_missing_receipts(scan_at, tf_at):
    gate = _adapter_gate(scan_at, tf_at)
    with patch.object(READINESS.time, "monotonic", return_value=10.0):
        assert not gate._stage_ready(deadline=20.0)
