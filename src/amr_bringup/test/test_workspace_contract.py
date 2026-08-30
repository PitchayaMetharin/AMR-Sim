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
    assert contract["actions"]["/amr/control/dock_egress"] == {
        "type": "nav2_msgs/action/BackUp",
        "server": "amr_control/command_arbitration_node",
    }
    assert contract["actions"]["/amr/mission/navigate_to_pose_precise"] == {
        "type": "nav2_msgs/action/NavigateToPose",
        "server": "amr_mission/mission_supervisor_node",
    }
    assert contract["actions"]["/amr/smooth_path"] == {
        "type": "nav2_msgs/action/SmoothPath",
        "server": "amr_navigation/smoother_server",
    }
    assert contract["actions"]["/amr/manipulation/manipulate_product"] == {
        "type": "amr_interfaces/action/ManipulateProduct",
        "server": "amr_manipulation/manipulation_supervisor_node",
    }
    assert contract["actions"]["/amr/factory/transport_product"] == {
        "type": "amr_interfaces/action/TransportProduct",
        "server": "amr_factory/factory_supervisor_node",
    }
    assert contract["services"]["/amr/factory/set_operation_mode"] == {
        "type": "amr_interfaces/srv/SetOperationMode",
        "server": "amr_factory/factory_supervisor_node",
    }
    assert topics["/amr/control/cmd_vel"] == {
        "type": "geometry_msgs/msg/TwistStamped",
        "publisher": "amr_control/command_arbitration_node",
    }
    assert topics["/amr/mpc/cmd_vel"] == {
        "type": "geometry_msgs/msg/Twist",
        "publisher": "nav2_controller/controller_server",
    }
    assert topics["/amr/health/status"] == {
        "type": "amr_interfaces/msg/HealthStatus",
        "publisher": "amr_health/health_supervisor_node",
    }
    assert topics["/amr/factory/status"] == {
        "type": "amr_interfaces/msg/FactoryStatus",
        "publisher": "amr_factory/factory_supervisor_node",
    }
    assert topics["/amr/manipulation/status"] == {
        "type": "amr_interfaces/msg/ManipulatorStatus",
        "publisher": "amr_manipulation/manipulation_supervisor_node",
    }
    assert topics["/amr/sensors/front_lidar/scan"]["publisher"] != topics[
        "/amr/sensors/rear_lidar/scan"
    ]["publisher"]
    assert topics["/amr/perception/front_lidar/points"]["publisher"] != topics[
        "/amr/perception/rear_lidar/points"
    ]["publisher"]
    assert topics["/amr/sensors/product_camera/image_rect"] == {
        "type": "sensor_msgs/msg/Image",
        "publisher": "amr_sensor_adapters/product_camera_adapter_node",
    }
    assert topics["/amr/sensors/product_camera/camera_info"] == {
        "type": "sensor_msgs/msg/CameraInfo",
        "publisher": "amr_sensor_adapters/product_camera_adapter_node",
    }
    assert topics["/amr/sensors/product_camera/depth"] == {
        "type": "sensor_msgs/msg/Image",
        "publisher": "amr_sensor_adapters/product_camera_adapter_node",
    }
    assert topics["/amr/perception/product_tags"] == {
        "type": "apriltag_msgs/msg/AprilTagDetectionArray",
        "publisher": "apriltag_ros/product_tag_detector",
    }
    assert topics["/amr/localization/wheel_odometry"]["publisher"] == (
        "amr_localization/wheel_odometry_node"
    )
    assert topics["/amr/localization/odometry"]["publisher"] == (
        "robot_localization/ekf_filter_node"
    )
    assert topics["/tf:odom->base_footprint"]["publisher"] == (
        "robot_localization/ekf_filter_node"
    )
    assert topics["/tf:map->odom"]["publisher"] == (
        "slam_toolbox/async_slam_toolbox_node"
    )
    assert contract["runtime_modes"]["factory"]["/tf:map->odom"] == (
        "nav2_amcl/amcl"
    )
    assert all(isinstance(spec["publisher"], str) for spec in topics.values())
    unstamped_commands = {
        name for name, spec in topics.items()
        if spec["type"] == "geometry_msgs/msg/Twist"
    }
    assert unstamped_commands == {"/amr/mpc/cmd_vel"}


def test_workspace_contains_authorized_packages():
    package_names = sorted(
        path.name
        for path in WORKSPACE_SRC.iterdir()
        if (path / "package.xml").is_file()
    )
    assert package_names == [
        "amr_base_adapter",
        "amr_bringup",
        "amr_control",
        "amr_description",
        "amr_exploration",
        "amr_factory",
        "amr_health",
        "amr_interfaces",
        "amr_localization",
        "amr_manipulation",
        "amr_mission",
        "amr_mpc_controller",
        "amr_navigation",
        "amr_perception",
        "amr_sensor_adapters",
        "amr_simulation",
        "amr_slam",
    ]
