"""Blockage-limit INCOMPLETE must satisfy the unchanged AWS terminal contract
(root-authored acceptance test, 2026-10-09; implementer must not edit).

Hospital run 17 stopped safely after 10 consecutive confirmed blockages, but the
reason "repeated obstacle blockage: ..." lacked the "exploration incomplete:"
prefix that aws_exploration_monitor.terminal_evidence requires of every
INCOMPLETE without an explicit acknowledgement (every other explorer INCOMPLETE
path already uses it).  The fix is the prefix only: same body, limit, state and
counts; the monitor stays unchanged.
"""
from pathlib import Path
import sys

TEST_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TEST_DIR.parent / "scripts"))
sys.path.insert(0, str(TEST_DIR))
sys.path.insert(0, str(TEST_DIR.parents[1] / "amr_simulation" / "scripts"))

import aws_exploration_monitor as monitor  # noqa: E402
import frontier_explorer as fe  # noqa: E402
import test_frontier_approach as approach  # noqa: E402
import test_frontier_lifecycle as lifecycle  # noqa: E402

EXPECTED_REASON = (
    "exploration incomplete: repeated obstacle blockage: %d consecutive "
    "confirmed blockages without reaching a goal" % fe.MAX_CONSECUTIVE_BLOCKAGES)


def _blocked_out_node():
    node = lifecycle._selection_fixture(lifecycle._node(autostart=True))
    for index in range(fe.MAX_CONSECUTIVE_BLOCKAGES):
        approach._confirmed_blockage(node, (14.5 + 0.1 * index, 10.5))
    return node


def _emitted_status(node):
    with node._lock:
        node._publish_status_locked()
    return node.status_pub.messages[-1].status[0]


def test_blockage_limit_reason_carries_the_incomplete_prefix():
    node = _blocked_out_node()
    assert node.state == "INCOMPLETE"
    assert node.fault_latched is False
    assert node.reason == EXPECTED_REASON


def test_unchanged_aws_monitor_accepts_the_blockage_limit_terminal():
    node = _blocked_out_node()
    verdict = monitor.terminal_evidence(_emitted_status(node))
    assert verdict is not None, "AWS terminal contract rejected the blockage-limit INCOMPLETE"
    assert verdict["state"] == "INCOMPLETE"
    assert verdict["reason"] == EXPECTED_REASON
    assert verdict["fault_latched"] is False


def test_below_the_limit_there_is_no_terminal_verdict():
    node = lifecycle._selection_fixture(lifecycle._node(autostart=True))
    for index in range(fe.MAX_CONSECUTIVE_BLOCKAGES - 1):
        approach._confirmed_blockage(node, (14.5 + 0.1 * index, 10.5))
    assert node.state == "RECOVERY_WAIT"
    assert monitor.terminal_evidence(_emitted_status(node)) is None
