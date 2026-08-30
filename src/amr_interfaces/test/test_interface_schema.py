from pathlib import Path


INTERFACE_ROOT = Path(__file__).resolve().parents[1]
STATUS_FILES = (
    "BaseStatus.msg",
    "HealthStatus.msg",
)


def test_status_interfaces_are_fail_closed_and_traceable():
    required_fields = {
        "std_msgs/Header header",
        "uint32 sequence",
        "bool valid",
        "uint32 source_boot_id",
        "uint8 state",
        "uint16 reason",
    }
    for name in STATUS_FILES:
        text = (INTERFACE_ROOT / "msg" / name).read_text()
        assert required_fields <= set(text.splitlines())
        assert "uint8 UNKNOWN=0" in text
        assert "uint16 REASON_UNAVAILABLE=0" in text


def test_health_status_is_observational_and_fail_closed():
    text = (INTERFACE_ROOT / "msg" / "HealthStatus.msg").read_text()
    assert "bool base_ready" in text
    assert "bool raw_permission" not in text
    assert "uint8 HEALTHY=2" in text
    assert "uint16 REASON_EVIDENCE_MISSING_OR_STALE=2" in text


def test_manipulator_status_encodes_fail_closed_transport_state():
    lines = set((INTERFACE_ROOT / "msg" / "ManipulatorStatus.msg").read_text().splitlines())
    assert {
        "uint8 STARTING=0",
        "uint8 STOWED_EMPTY=1",
        "uint8 STOWED_LOADED=2",
        "uint8 MOVING=3",
        "uint8 DEPLOYED=4",
        "uint8 FAULT=5",
        "uint32 source_boot_id",
        "uint32 sequence",
        "bool valid",
        "uint8 state",
        "bool base_motion_allowed",
        "bool product_attached",
        "string product_id",
    } <= lines


def test_phase14_factory_interfaces_match_the_action_and_status_contracts():
    manipulate = (INTERFACE_ROOT / "action" / "ManipulateProduct.action").read_text()
    assert "uint8 PICK=1" in manipulate
    assert "uint8 PLACE=2" in manipulate
    assert "uint8 ATTACHMENT_FAILED=6" in manipulate
    assert manipulate.count("---") == 2

    transport = (INTERFACE_ROOT / "action" / "TransportProduct.action").read_text()
    assert "bool delivered" in transport
    assert "uint8 DEPENDENCY_UNAVAILABLE=6" in transport
    assert "uint32 queue_position" in transport
    assert transport.count("---") == 2

    operation_mode = (INTERFACE_ROOT / "srv" / "SetOperationMode.srv").read_text()
    assert {"uint8 MANUAL=0", "uint8 AUTONOMOUS=1", "bool accepted"} <= set(
        operation_mode.splitlines())

    factory_status = set(
        (INTERFACE_ROOT / "msg" / "FactoryStatus.msg").read_text().splitlines())
    assert {
        "std_msgs/Header header",
        "uint32 sequence",
        "uint8 mode",
        "uint8 phase",
        "bool active",
        "uint32 queue_depth",
        "string pickup_station_id",
        "string destination_station_id",
        "string product_id",
        "bool product_attached",
        "uint8 last_outcome",
        "string detail",
    } <= factory_status
