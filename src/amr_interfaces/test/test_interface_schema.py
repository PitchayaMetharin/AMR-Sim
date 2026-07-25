from pathlib import Path


INTERFACE_ROOT = Path(__file__).resolve().parents[1]
STATUS_FILES = (
    "BaseStatus.msg",
    "GateStatus.msg",
    "PlcConnectionStatus.msg",
    "PlcState.msg",
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


def test_permission_bearing_interfaces_expose_raw_permission():
    for name in ("GateStatus.msg", "PlcState.msg"):
        text = (INTERFACE_ROOT / "msg" / name).read_text()
        assert "bool raw_permission" in text


def test_gateway_services_do_not_claim_plc_authority():
    for name in ("RequestMotionEnable.srv", "RequestReset.srv"):
        text = (INTERFACE_ROOT / "srv" / name).read_text()
        assert "uint16 REASON_UNAVAILABLE=0" in text
        assert "bool accepted_for_delivery" in text
        assert "bool drive_permission" not in text
