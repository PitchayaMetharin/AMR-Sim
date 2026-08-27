"""Run one ordered Gate 6 product-mass grasp/transport/place stage."""

import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _finite_pose(value, label, keys=("x", "y", "yaw")):
    if value is None or any(key not in value for key in keys):
        raise RuntimeError(f"registry pose is missing {label}")
    result = [float(value[key]) for key in keys]
    if not all(math.isfinite(item) for item in result):
        raise RuntimeError(f"registry pose is non-finite: {label}")
    return result


def _validate_pickup_egress(dock, egress, approach, label):
    if not all(math.isfinite(value) for value in (*dock, *egress, *approach)):
        raise RuntimeError(f"registry egress geometry is non-finite: {label}")
    dock_to_egress = (egress[0] - dock[0], egress[1] - dock[1])
    dock_to_approach = (approach[0] - dock[0], approach[1] - dock[1])
    cross = (dock_to_egress[0] * dock_to_approach[1]
             - dock_to_egress[1] * dock_to_approach[0])
    dot = (dock_to_egress[0] * dock_to_approach[0]
           + dock_to_egress[1] * dock_to_approach[1])
    approach_distance_sq = dock_to_approach[0] ** 2 + dock_to_approach[1] ** 2
    reverse_distance = math.hypot(*dock_to_egress)
    if abs(cross) > 1e-9 or not (0.0 < dot < approach_distance_sq):
        raise RuntimeError(f"egress must be collinear between dock and approach: {label}")
    if not math.isclose(reverse_distance, 0.50, abs_tol=1e-9):
        raise RuntimeError(f"egress reverse distance must be 0.50 m: {label}")
    if not math.isclose(dock[2], egress[2], abs_tol=1e-9) or not math.isclose(
            dock[2], approach[2], abs_tol=1e-9):
        raise RuntimeError(f"egress yaw must match dock and approach: {label}")


def _resolve_registry(context):
    factory = get_package_share_directory("amr_factory")
    control = get_package_share_directory("amr_control")
    with open(os.path.join(factory, "config", "products.yaml"), encoding="utf-8") as stream:
        products_registry = yaml.safe_load(stream)
    with open(os.path.join(factory, "config", "stations.yaml"), encoding="utf-8") as stream:
        stations_registry = yaml.safe_load(stream)
    with open(os.path.join(control, "config", "control.yaml"), encoding="utf-8") as stream:
        control_registry = yaml.safe_load(stream)

    products = products_registry.get("products", {})
    stations = stations_registry.get("stations", {})
    control_parameters = control_registry.get(
        "/amr/command_arbitration_node", {}).get("ros__parameters", {})
    egress_speed = float(control_parameters.get("egress_max_speed_mps", float("nan")))
    egress_limit = float(control_parameters.get("egress_time_limit_s", float("nan")))
    egress_max_distance = float(
        control_parameters.get("egress_max_distance_m", float("nan")))
    if not all(math.isfinite(value) and value > 0.0 for value in (
            egress_speed, egress_limit, egress_max_distance)):
        raise RuntimeError("control egress limits are missing or invalid")
    slots = products_registry.get("dispatch_slots", [])
    if not isinstance(products, dict) or not isinstance(stations, dict):
        raise RuntimeError("registry products and stations must be mappings")
    if len(products) != 3 or len(slots) != len(products):
        raise RuntimeError("mismatched product/dispatch-slot count")
    if len({slot.get("id") for slot in slots}) != len(slots):
        raise RuntimeError("duplicate dispatch slot ID")

    by_id = {}
    tag_ids = set()
    for model, entry in products.items():
        if not isinstance(entry, dict) or "tag_id" not in entry:
            raise RuntimeError(f"product {model} is missing a tag ID")
        tag_id = int(entry["tag_id"])
        if tag_id in tag_ids:
            raise RuntimeError(f"duplicate product tag ID {tag_id}")
        tag_ids.add(tag_id)
        pickup_name = entry.get("pickup_station")
        station = stations.get(pickup_name)
        if station is None or station.get("dock") is None:
            raise RuntimeError(f"product {model} references a missing pickup station")
        mass = float(entry.get("mass"))
        if not math.isfinite(mass) or mass < 0.0:
            raise RuntimeError(f"product {model} has an invalid mass")
        by_id[tag_id] = (model, entry, station)

    dispatch = stations.get("dispatch")
    if dispatch is None or dispatch.get("dock") is None:
        raise RuntimeError("dispatch station is missing")
    dispatch_approach = _finite_pose(dispatch.get("approach"), "dispatch approach")
    dispatch_dock = _finite_pose(dispatch.get("dock"), "dispatch dock")

    slot_by_id = {}
    for slot in slots:
        if not isinstance(slot, dict) or not isinstance(slot.get("id"), str):
            raise RuntimeError("dispatch slot is missing an ID")
        position = [float(slot.get(axis)) for axis in ("x", "y", "z")]
        if not all(math.isfinite(item) for item in position):
            raise RuntimeError(f"dispatch slot {slot.get('id')} has a non-finite pose")
        slot_by_id[slot["id"]] = position

    assignment = {101: "dispatch_1", 102: "dispatch_2", 103: "dispatch_3"}
    raw_id = int(LaunchConfiguration("product_id").perform(context))
    if raw_id not in assignment or raw_id not in by_id:
        raise RuntimeError(f"unknown product_id {raw_id}")
    selected_slot_id = assignment[raw_id]
    if selected_slot_id not in slot_by_id:
        raise RuntimeError(f"missing dispatch slot {selected_slot_id}")

    model, entry, pickup_station = by_id[raw_id]
    size = products_registry.get("product_size")
    if not isinstance(size, list) or len(size) != 3 or not all(
            math.isfinite(float(value)) and float(value) > 0.0 for value in size):
        raise RuntimeError("product_size registry value is invalid")
    all_slots = [slot_by_id[f"dispatch_{index}"] for index in (1, 2, 3)]
    pickup_station_pose = _finite_pose(
        pickup_station.get("approach"), f"{entry['pickup_station']}.approach")
    pickup_dock_pose = _finite_pose(
        pickup_station.get("dock"), f"{entry['pickup_station']}.dock")
    pickup_egress_pose = _finite_pose(
        pickup_station.get("egress"), f"{entry['pickup_station']}.egress")
    _validate_pickup_egress(
        pickup_dock_pose, pickup_egress_pose, pickup_station_pose,
        f"{entry['pickup_station']}.egress")
    reverse_distance = math.hypot(
        pickup_egress_pose[0] - pickup_dock_pose[0],
        pickup_egress_pose[1] - pickup_dock_pose[1])
    if reverse_distance > egress_max_distance:
        raise RuntimeError("registered egress exceeds the arbitration distance limit")
    pickup_approach_distance = math.hypot(
        pickup_station_pose[0] - pickup_egress_pose[0],
        pickup_station_pose[1] - pickup_egress_pose[1])
    if not math.isfinite(pickup_approach_distance) or pickup_approach_distance <= 0.0:
        raise RuntimeError("registered pickup approach reverse distance is invalid")
    if pickup_approach_distance > egress_max_distance:
        raise RuntimeError(
            "registered pickup approach reverse exceeds the arbitration distance limit")
    return {
        "product_id": raw_id,
        "product_model": model,
        "product_mass_kg": float(entry["mass"]),
        "product_size": [float(value) for value in size],
        "pickup_station_pose": pickup_station_pose,
        "pickup_dock_pose": pickup_dock_pose,
        "pickup_egress_pose": pickup_egress_pose,
        "dock_egress_speed_mps": egress_speed,
        "dock_egress_time_limit_s": egress_limit,
        "dock_egress_max_distance_m": egress_max_distance,
        "dispatch_approach_pose": dispatch_approach,
        "dispatch_dock_pose": dispatch_dock,
        "dispatch_slot_1_position": all_slots[0],
        "dispatch_slot_2_position": all_slots[1],
        "dispatch_slot_3_position": all_slots[2],
        "selected_dispatch_slot_id": selected_slot_id,
        "selected_dispatch_slot_position": slot_by_id[selected_slot_id],
        "selected_dispatch_slot_index": int(selected_slot_id[-1]) - 1,
    }


def _make_node(context, moveit_config):
    registry = _resolve_registry(context)
    return [Node(
        package="amr_manipulation",
        executable="gate6_mass_stage",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": True, **registry}],
        remappings=[("joint_states", "/amr/base/joint_states")],
    )]


def generate_launch_description():
    description = get_package_share_directory("amr_description")
    manipulation = get_package_share_directory("amr_manipulation")
    config = (
        MoveItConfigsBuilder(
            "phase14_mobile_manipulator", package_name="amr_manipulation")
        .robot_description(
            file_path=os.path.join(
                description, "urdf", "phase14_mobile_manipulator.urdf.xacro"),
            mappings={
                "controller_config": os.path.join(
                    description, "config", "phase14_mobile_manipulator_controllers.yaml"),
                "loaded_product": "false",
                "factory_attachment": "true",
                "joint_state_topic": "/world/factory_world/model/amr/joint_state",
            },
        )
        .robot_description_semantic(file_path=os.path.join(
            description, "config", "phase14_mobile_manipulator.srdf"))
        .robot_description_kinematics(file_path=os.path.join(
            manipulation, "config", "kinematics.yaml"))
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )
    return LaunchDescription([
        DeclareLaunchArgument("product_id", default_value="101"),
        OpaqueFunction(function=lambda context: _make_node(context, config)),
    ])
