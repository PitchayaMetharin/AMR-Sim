"""Launch the Phase 14 autonomous factory graph."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_autonomous_mapping(factory):
    """Build adapter parameters from the validated factory registries."""
    config_dir = os.path.join(factory, "config")
    with open(os.path.join(config_dir, "products.yaml"), encoding="utf-8") as stream:
        products = yaml.safe_load(stream) or {}
    with open(os.path.join(config_dir, "stations.yaml"), encoding="utf-8") as stream:
        stations = yaml.safe_load(stream) or {}

    raw_products = products.get("products")
    raw_stations = stations.get("stations")
    if not isinstance(raw_products, dict) or not isinstance(raw_stations, dict):
        raise RuntimeError("autonomous factory registries must contain mappings")

    autonomous = []
    for model, entry in raw_products.items():
        if not isinstance(model, str) or not isinstance(entry, dict):
            raise RuntimeError("product registry contains an invalid entry")
        if entry.get("autonomous_enabled") is not True:
            continue
        station_id = entry.get("pickup_station")
        product_id = entry.get("tag_id")
        if (not isinstance(station_id, str) or not station_id or
                isinstance(product_id, bool) or not isinstance(product_id, int) or
                product_id <= 0):
            raise RuntimeError(f"autonomous product {model} has an invalid mapping")
        station = raw_stations.get(station_id)
        if not isinstance(station, dict) or station.get("role") != "pickup":
            raise RuntimeError(
                f"autonomous product {model} references a non-pickup station")
        autonomous.append((station_id, product_id))

    dispatch_stations = sorted(
        station_id for station_id, entry in raw_stations.items()
        if isinstance(station_id, str) and isinstance(entry, dict) and
        entry.get("role") == "dispatch")
    if not autonomous or not dispatch_stations:
        raise RuntimeError("autonomous factory registries have no usable route")

    autonomous.sort()
    return (
        [f"{station_id}={product_id}" for station_id, product_id in autonomous],
        [product_id for _, product_id in autonomous],
        dispatch_stations,
    )


def _validate_autonomous_mode(context):
    if LaunchConfiguration("control_mode").perform(context).strip() != "autonomous":
        raise RuntimeError("factory_autonomous.launch.py requires control_mode=autonomous")


def generate_launch_description():
    factory = get_package_share_directory("amr_factory")
    products_config = os.path.join(factory, "config", "products.yaml")
    stations_config = os.path.join(factory, "config", "stations.yaml")
    product_station_map, autonomous_product_ids, dispatch_station_ids = (
        _load_autonomous_mapping(factory)
    )
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(factory, "launch", "factory_localization.launch.py")
        ),
        launch_arguments={
            "headless": LaunchConfiguration("headless"),
            "software_rendering": LaunchConfiguration("software_rendering"),
            "require_hardware_rendering": LaunchConfiguration(
                "require_hardware_rendering"),
            "factory_attachment": LaunchConfiguration("factory_attachment"),
            "control_mode": LaunchConfiguration("control_mode"),
            "mapping_mode": "false",
            "map_yaml": LaunchConfiguration("map_yaml"),
            "initial_x": LaunchConfiguration("initial_x"),
            "initial_y": LaunchConfiguration("initial_y"),
            "initial_yaw": LaunchConfiguration("initial_yaw"),
        }.items(),
    )
    manipulation = Node(
        package="amr_manipulation",
        executable="cycle_manipulation_supervisor.py",
        name="manipulation_supervisor_node",
        namespace="/amr",
        parameters=[{
            "use_sim_time": True,
            "product_station_map": product_station_map,
            "autonomous_product_ids": autonomous_product_ids,
            "dispatch_station_ids": dispatch_station_ids,
            "internal_status_topic": "/amr/manipulation/internal/status",
            "internal_cancel_service": "/amr/manipulation/internal/cancel_cycle_motion",
            "bootstrap_service": "/amr/simulation/attachment_bootstrap/verify",
        }],
        output="screen",
    )
    factory_supervisor = Node(
        package="amr_factory",
        executable="factory_supervisor_node",
        name="factory_supervisor_node",
        namespace="/amr",
        parameters=[{
            "use_sim_time": True,
            "products_config": products_config,
            "stations_config": stations_config,
        }],
        output="screen",
    )
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="true"),
        DeclareLaunchArgument("software_rendering", default_value="false"),
        DeclareLaunchArgument("require_hardware_rendering", default_value="true"),
        DeclareLaunchArgument("factory_attachment", default_value="false"),
        DeclareLaunchArgument("control_mode", default_value="autonomous"),
        DeclareLaunchArgument(
            "map_yaml", default_value=os.path.join(factory, "maps", "factory.yaml")),
        DeclareLaunchArgument("initial_x", default_value="-4.5"),
        DeclareLaunchArgument("initial_y", default_value="0.0"),
        DeclareLaunchArgument("initial_yaw", default_value="0.0"),
        OpaqueFunction(function=_validate_autonomous_mode),
        localization,
        manipulation,
        factory_supervisor,
    ])
