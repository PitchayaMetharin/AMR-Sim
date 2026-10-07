"""Launch the Phase 14 Nav2 regulated pure pursuit controller and local costmap."""
import copy
import math
import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _registry(path):
    with open(path, encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise ValueError("Final-position registry must be a mapping")
    return value


def _coordinates(value, keys):
    if not isinstance(value, dict):
        raise ValueError("Final-position coordinates must be a mapping")
    result = []
    for key in keys:
        number = value.get(key)
        if type(number) not in (int, float) or not math.isfinite(number):
            raise ValueError("Final-position coordinates must be finite numbers")
        result.append(float(number))
    return result


def build_controller_overrides(parameters, enabled, products_registry, stations_registry):
    # Disabled launches retain the public YAML verbatim and read no registries.
    if not enabled:
        return {}
    products = _registry(products_registry)
    stations = _registry(stations_registry)
    if products.get("frame_id") != "map" or stations.get("frame_id") != "map":
        raise ValueError("Final-position registries must use map")
    records = products.get("products")
    station_records = stations.get("stations")
    slots = products.get("dispatch_slots")
    if not isinstance(records, dict) or not isinstance(station_records, dict) or not isinstance(slots, list):
        raise ValueError("Missing final-position registry records")
    by_slot = {}
    for slot in slots:
        if not isinstance(slot, dict) or not isinstance(slot.get("id"), str) or not slot["id"] or slot["id"] in by_slot:
            raise ValueError("Dispatch slot IDs must be unique strings")
        by_slot[slot["id"]] = slot
    ids, assigned_slots = set(), set()
    for record in records.values():
        if not isinstance(record, dict) or type(record.get("tag_id")) is not int or record["tag_id"] in ids:
            raise ValueError("Product IDs must be unique nonbool integers")
        ids.add(record["tag_id"])
        slot_id = record.get("dispatch_slot")
        pickup = record.get("pickup_station")
        if not isinstance(slot_id, str) or slot_id not in by_slot or slot_id in assigned_slots:
            raise ValueError("Product dispatch slots must exist and cannot be shared")
        assigned_slots.add(slot_id)
        if not isinstance(pickup, str) or not isinstance(station_records.get(pickup), dict) or station_records[pickup].get("role") != "pickup":
            raise ValueError("Registered pickup station must exist")
    for name, tag_id, slot_id in (("product_a", 101, "dispatch_1"), ("product_b", 102, "dispatch_2")):
        record = records.get(name)
        if not isinstance(record, dict) or record.get("tag_id") != tag_id or record.get("dispatch_slot") != slot_id or record.get("autonomous_enabled") is not True:
            raise ValueError("Private A/B product ID and slot binding is invalid")
    dispatch = station_records.get("dispatch")
    if not isinstance(dispatch, dict) or dispatch.get("role") != "dispatch":
        raise ValueError("Registered dispatch station must exist")
    raw = {
        "final_position_profiles.dispatch_dock": _coordinates(dispatch.get("dock"), ("x", "y", "yaw")),
        "final_position_profiles.slot_a": _coordinates(by_slot["dispatch_1"], ("x", "y", "z")),
        "final_position_profiles.slot_b": _coordinates(by_slot["dispatch_2"], ("x", "y", "z")),
    }
    controller = _registry(parameters)["/amr/controller_server"]["ros__parameters"]
    raw["controller_plugins"] = list(controller["controller_plugins"])
    for suffix in ("A", "B"):
        for base, name, plugin in (
            ("FollowPath", "FinalPositionFollowPath", "amr_mpc_controller::FinalPositionRPP"),
            ("PlacementFollowPath", "FinalPositionPlacementFollowPath", "amr_mpc_controller::FinalPositionPlacementRPP"),
        ):
            profile = copy.deepcopy(controller[base])
            profile.update(plugin=plugin, desired_linear_vel=0.50)
            raw[name + suffix] = profile
            raw["controller_plugins"].append(name + suffix)
    return raw


def _launch_controller(context):
    parameters = os.path.join(
        get_package_share_directory("amr_mpc_controller"),
        "config",
        "controller.yaml",
    )
    overrides = build_controller_overrides(
        parameters,
        IfCondition(LaunchConfiguration("enable_final_position_profiles")).evaluate(context),
        LaunchConfiguration("products_registry").perform(context),
        LaunchConfiguration("stations_registry").perform(context),
    )
    # Humble's lifecycle manager starts autostart from a zero-delay wall
    # timer.  The controller server constructs its nested local costmap on
    # startup, so launching both processes together can send the first
    # change_state request before the controller has finished constructing its
    # lifecycle services.  Keep the controller process first and give that
    # construction a bounded one-second barrier before starting its manager.
    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        namespace="/amr",
        output="screen",
        parameters=[parameters, {
            "controller_frequency": ParameterValue(
                LaunchConfiguration("controller_frequency"), value_type=float),
        }, overrides],
        remappings=[("cmd_vel", "/amr/mpc/cmd_vel")],
    )
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_controller",
        namespace="/amr",
        output="screen",
        parameters=[parameters],
    )
    return [
        controller_server,
        TimerAction(period=1.0, actions=[lifecycle_manager]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("controller_frequency", default_value="20.0"),
        DeclareLaunchArgument("enable_final_position_profiles", default_value="false"),
        DeclareLaunchArgument("products_registry", default_value=""),
        DeclareLaunchArgument("stations_registry", default_value=""),
        OpaqueFunction(function=_launch_controller),
    ])
