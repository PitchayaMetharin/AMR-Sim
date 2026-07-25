import json
from pathlib import Path


BRINGUP_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_SRC = BRINGUP_ROOT.parent


def test_localhost_only_environment_is_declared():
    text = (BRINGUP_ROOT / "env" / "amr_ros_env.sh").read_text()
    assert "ROS_DOMAIN_ID=1" in text
    assert "ROS_LOCALHOST_ONLY=1" in text


def test_runtime_defaults_use_one_robot_namespace_and_sim_time():
    text = (BRINGUP_ROOT / "config" / "runtime_defaults.yaml").read_text()
    assert "robot_namespace: /amr" in text
    assert "use_sim_time: true" in text


def test_qos_profiles_match_the_phase_four_contract():
    profiles = json.loads((BRINGUP_ROOT / "config" / "qos_profiles.yaml").read_text())
    assert profiles["clock"] == {
        "depth": 1,
        "reliability": "best_effort",
        "durability": "volatile",
    }
    assert profiles["sensor"]["depth"] == 5
    assert profiles["sensor"]["reliability"] == "best_effort"
    assert profiles["authority"]["deadline_ms"] == 100
    assert profiles["command"]["deadline_ms"] == 100
    assert profiles["command"]["lifespan_ms"] == 200
    assert profiles["static_tf"]["durability"] == "transient_local"


def test_canonical_interfaces_have_one_named_authority():
    contract = json.loads((BRINGUP_ROOT / "config" / "interface_ownership.yaml").read_text())
    topics = contract["topics"]

    assert contract["namespace"] == "/amr"
    assert topics["/amr/control/cmd_vel_request"] == {
        "type": "geometry_msgs/msg/TwistStamped",
        "publisher": "amr_control/command_arbitration_node",
    }
    assert topics["/amr/control/cmd_vel_gated"] == {
        "type": "geometry_msgs/msg/TwistStamped",
        "publisher": "amr_control/motion_gate_node",
    }
    assert topics["/amr/plc/state"]["publisher"] == "amr_plc_gateway/plc_gateway_node"
    assert topics["/amr/sensors/front_lidar/scan"]["publisher"] != topics[
        "/amr/sensors/rear_lidar/scan"
    ]["publisher"]
    assert all(isinstance(spec["publisher"], str) for spec in topics.values())
    assert all(spec["type"] != "geometry_msgs/msg/Twist" for spec in topics.values())


def test_only_phase_four_packages_are_created():
    package_names = sorted(
        path.name
        for path in WORKSPACE_SRC.iterdir()
        if (path / "package.xml").is_file()
    )
    assert package_names == ["amr_bringup", "amr_interfaces"]
