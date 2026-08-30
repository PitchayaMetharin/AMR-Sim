import math
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def test_factory_world_is_local_sdf_19_with_required_asset_categories():
    world_path = ROOT / "worlds" / "factory.sdf"
    world_text = world_path.read_text()
    sdf = ET.fromstring(world_text)
    assert sdf.attrib["version"] == "1.9"
    assert sdf.find("world").attrib["name"] == "factory_world"
    assert "http://" not in world_text
    assert "https://" not in world_text
    assert "fuel" not in world_text.lower()
    uris = {uri.text for uri in sdf.findall(".//include/uri")}
    assert uris == {
        "model://aws_robomaker_warehouse_ShelfD_01",
        "model://aws_robomaker_warehouse_ClutteringA_01",
        "model://aws_robomaker_warehouse_PalletJackB_01",
        "model://aws_robomaker_warehouse_Bucket_01",
    }
    assert all((ROOT / "models" / uri.removeprefix("model://")).is_dir()
               for uri in uris)


def test_factory_world_experimental_step_and_rendering_contract():
    sdf = ET.fromstring((ROOT / "worlds" / "factory.sdf").read_text())
    world = sdf.find("world")
    physics = world.find("./physics")
    max_step = float(physics.find("max_step_size").text)
    assert physics.attrib["name"] == "3.333333ms"
    assert max_step == 0.0033333333333333335
    for rate_hz, expected_steps, rate_name in (
        (100.0, 3, "controller/contact"),
        (10.0, 30, "lidar/camera"),
    ):
        steps_per_cycle = 1.0 / (rate_hz * max_step)
        assert math.isclose(steps_per_cycle, expected_steps, abs_tol=1e-12), (
            f"{rate_name} cycle must be integral at {expected_steps} steps"
        )
    assert float(world.find("./physics/real_time_factor").text) == 1.0
    assert world.find("./scene/shadows").text == "false"
    assert world.find("./light/cast_shadows").text == "false"


def test_aws_assets_retain_license_attribution_and_only_use_local_meshes():
    assert "Permission is hereby granted" in (
        ROOT / "assets" / "AWS_SMALL_WAREHOUSE_LICENSE").read_text()
    attribution = (ROOT / "assets" / "AWS_SMALL_WAREHOUSE_ATTRIBUTION.md").read_text()
    assert "ee0af733315e78432408c3cd98d378ecee5f767c" in attribution
    for model in (ROOT / "models").iterdir():
        model_text = (model / "model.sdf").read_text()
        assert "file://" not in model_text
        assert "http://" not in model_text
        for uri in ET.fromstring(model_text).findall(".//uri"):
            assert uri.text.startswith(f"model://{model.name}/")


def test_station_registry_is_the_exact_phase14_pose_and_tag_contract():
    registry = yaml.safe_load((ROOT / "config" / "stations.yaml").read_text())
    assert registry["frame_id"] == "map"
    assert registry["tag_family"] == "36h11"
    assert registry["station_tag_size"] == 0.10
    assert registry["max_hamming"] == 0
    expected = {
        "home": ((-4.5, 0.0, 0.0), None, None, None),
        "pickup_a": ((1.5, 3.0, 0.0), (2.4, 3.0, 0.0),
                     (1.9, 3.0, 0.0), 10),
        "pickup_b": ((1.5, 0.0, 0.0), (2.4, 0.0, 0.0),
                     (1.9, 0.0, 0.0), 11),
        "pickup_c": ((1.5, -3.0, 0.0), (2.4, -3.0, 0.0),
                     (1.9, -3.0, 0.0), 12),
        "dispatch": ((-2.5, 0.0, 3.141592653589793),
                     (-3.4, 0.0, 3.141592653589793), None, 20),
    }
    for name, expected_entry in expected.items():
        approach, dock, egress, tag_id = expected_entry
        station = registry["stations"][name]
        assert tuple(station["approach"].values()) == approach
        assert (None if station["dock"] is None
                else tuple(station["dock"].values())) == dock
        assert (None if station.get("egress") is None
                else tuple(station["egress"].values())) == egress
        assert station.get("tag_id") == tag_id


def test_product_registry_has_fixed_ids_masses_dimensions_and_dispatch_slots():
    registry = yaml.safe_load((ROOT / "config" / "products.yaml").read_text())
    assert registry["product_size"] == [0.30, 0.20, 0.15]
    assert registry["maximum_product_mass"] == 5.0
    assert registry["tag_to_grasp"] == {
        "translation_xyz": [0.0, -0.10, 0.156],
        "rotation_xyzw": [0.5, -0.5, 0.5, 0.5],
    }
    assert [(item["tag_id"], item["mass"])
            for item in registry["products"].values()] == [
                (101, 1.0), (102, 3.0), (103, 5.0)]
    assert len({slot["id"] for slot in registry["dispatch_slots"]}) == 3


def test_canonical_map_has_exact_geometry_and_metadata():
    metadata = yaml.safe_load((ROOT / "maps" / "factory.yaml").read_text())
    assert metadata["resolution"] == 0.05
    assert metadata["origin"] == [-6.0, -5.0, 0.0]
    with (ROOT / "maps" / "factory.pgm").open("rb") as image:
        assert image.readline().strip() == b"P5"
        dimensions = image.readline().strip()
        while dimensions.startswith(b"#"):
            dimensions = image.readline().strip()
        assert tuple(map(int, dimensions.split())) == (240, 200)
        assert int(image.readline()) == 255
        pixels = image.read()
    assert len(pixels) == 240 * 200
    assert 0 in pixels
    assert 255 in pixels


def test_registry_navigation_poses_have_required_map_clearance():
    metadata = yaml.safe_load((ROOT / "maps" / "factory.yaml").read_text())
    resolution = metadata["resolution"]
    origin_x, origin_y, _ = metadata["origin"]
    with (ROOT / "maps" / "factory.pgm").open("rb") as image:
        assert image.readline().strip() == b"P5"
        dimensions = image.readline().strip()
        while dimensions.startswith(b"#"):
            dimensions = image.readline().strip()
        width, height = map(int, dimensions.split())
        assert int(image.readline()) == 255
        pixels = image.read()

    registry = yaml.safe_load((ROOT / "config" / "stations.yaml").read_text())
    clearance_cells = math.ceil(0.65 / resolution)
    for station_name, station in registry["stations"].items():
        for pose_name in ("approach", "dock", "egress"):
            pose = station.get(pose_name)
            if pose is None:
                continue
            column = int((pose["x"] - origin_x) / resolution)
            map_row = int((pose["y"] - origin_y) / resolution)
            image_row = height - 1 - map_row
            for row_offset in range(-clearance_cells, clearance_cells + 1):
                for column_offset in range(-clearance_cells, clearance_cells + 1):
                    if math.hypot(row_offset, column_offset) > clearance_cells:
                        continue
                    row = image_row + row_offset
                    col = column + column_offset
                    assert 0 <= row < height and 0 <= col < width
                    assert pixels[row * width + col] != 0, (
                        f"{station_name}.{pose_name} lacks 0.65 m obstacle clearance"
                    )


def test_pickup_egress_geometry_is_collinear_bounded_and_reverse():
    registry = yaml.safe_load((ROOT / "config" / "stations.yaml").read_text())
    for name in ("pickup_a", "pickup_b", "pickup_c"):
        station = registry["stations"][name]
        dock = station["dock"]
        egress = station["egress"]
        approach = station["approach"]
        dock_to_egress = (egress["x"] - dock["x"], egress["y"] - dock["y"])
        dock_to_approach = (approach["x"] - dock["x"], approach["y"] - dock["y"])
        cross = (dock_to_egress[0] * dock_to_approach[1]
                 - dock_to_egress[1] * dock_to_approach[0])
        dot = (dock_to_egress[0] * dock_to_approach[0]
               + dock_to_egress[1] * dock_to_approach[1])
        distance = math.hypot(*dock_to_egress)
        approach_distance = math.hypot(*dock_to_approach)
        assert abs(cross) <= 1e-9
        assert 0.0 < dot < approach_distance * approach_distance
        assert math.isclose(distance, 0.50, abs_tol=1e-9)
        assert egress["yaw"] == dock["yaw"] == approach["yaw"]


def test_factory_localization_launch_uses_amcl_and_never_slam():
    launch = (ROOT / "launch" / "factory_localization.launch.py").read_text()
    assert 'package="nav2_amcl"' in launch
    assert 'package="nav2_map_server"' in launch
    assert "amr_slam" not in launch
    assert "slam_toolbox" not in launch
    assert "/world/factory_world/model/amr/joint_state" in launch
    assert 'remappings=[("joint_states", "/amr/base/joint_states")]' in launch
    assert "def launch_robot(context):" in launch
    assert 'DeclareLaunchArgument("factory_attachment", default_value="false")' in launch
    assert '"factory_attachment": factory_attachment' in launch
    assert "TimerAction(period=2.0, actions=[spawn])" in launch
    managed_start = launch.index("def managed_node")
    activate_handler = launch.index("activate = RegisterEventHandler", managed_start)
    return_order = launch.index("return [node, activate, configure]", managed_start)
    assert activate_handler < return_order
    adapter_start = launch.index("base_node, base_activate, base_configure")
    adapter_order = (
        '"amr_base_adapter", "base_adapter_node"',
        '"amr_sensor_adapters", "front_lidar_adapter_node"',
        '"amr_sensor_adapters", "rear_lidar_adapter_node"',
        '"amr_sensor_adapters", "imu_adapter_node"',
        '"amr_sensor_adapters", "product_camera_adapter_node"',
    )
    adapter_positions = [launch.index(token, adapter_start) for token in adapter_order]
    assert adapter_positions == sorted(adapter_positions)
    configure_timer = launch.index(
        "TimerAction(period=8.0, actions=[base_configure])", adapter_start)
    assert all(
        launch.index(token, adapter_start) < configure_timer
        for token in (
            "base_node, base_activate",
            "front_node, front_activate",
            "rear_node, rear_activate",
            "imu_node, imu_activate",
            "product_camera_node, product_camera_activate",
        )
    )
    assert "actions.extend(managed_node(package, executable))" not in launch
    assert launch.count('goal_state="active"') == 6
    for node_name, configure_name in (
        ("base_node", "front_configure"),
        ("front_node", "rear_configure"),
        ("rear_node", "imu_configure"),
        ("imu_node", "product_camera_configure"),
    ):
        handler_start = launch.index(f"target_lifecycle_node={node_name}", configure_timer)
        assert configure_timer < handler_start < launch.index(
            f"entities=[{configure_name}]", handler_start)
    adapter_ready_start = launch.index(
        "target_lifecycle_node=product_camera_node", configure_timer)
    adapter_ready_end = launch.index(
        "target_lifecycle_node=amcl", adapter_ready_start)
    adapter_ready_block = launch[adapter_ready_start:adapter_ready_end]
    assert "TimerAction(period=2.0, actions=[include_control])" in adapter_ready_block
    assert "TimerAction(period=4.0, actions=[localization_manager])" in adapter_ready_block
    amcl_ready_block = launch[adapter_ready_end:]
    assert "include_navigation" in amcl_ready_block
    assert "TimerAction(period=5.0, actions=[include_mpc])" in amcl_ready_block
    assert "TimerAction(period=8.0, actions=[include_mission])" in amcl_ready_block


def test_factory_gui_launch_defaults_to_hardware_and_retains_explicit_fallback():
    launch = (ROOT / "launch" / "factory_localization.launch.py").read_text()
    assert 'DeclareLaunchArgument("software_rendering", default_value="false")' in launch
    assert 'DeclareLaunchArgument("require_hardware_rendering", default_value="true")' in launch
    assert "accessible_render_devices" in launch
    assert "validate_hardware_rendering" in launch
    assert "software_renderer_forced" in launch
    assert 'server_arguments = ["-r", "-s", world_path]' in launch
    assert 'gui_arguments = [' in launch
    assert '"-g",' in launch
    assert '"--render-engine-gui", "ogre2"' in launch
    assert '"--render-engine-gui-api-backend", "opengl"' in launch
    assert 'period=2.0' in launch
    assert 'name="XDG_CACHE_HOME"' in launch
    assert 'name="XDG_CONFIG_HOME"' in launch
    assert 'name="XDG_RUNTIME_DIR"' in launch
    assert 'name="GZ_LOG_PATH"' in launch
    assert 'name="QT_X11_NO_MITSHM", value="1"' in launch
    assert 'name="LIBGL_ALWAYS_SOFTWARE", value="1"' in launch
    assert 'name="GALLIUM_DRIVER", value="llvmpipe"' in launch
    assert "os.chmod(runtime_dir, 0o700)" in launch
    assert 'SetEnvironmentVariable(name="HOME"' not in launch


def test_factory_launch_uses_async_fastdds_service_publication():
    launch = (ROOT / "launch" / "factory_localization.launch.py").read_text()
    setting = 'name="RMW_FASTRTPS_PUBLICATION_MODE", value="ASYNCHRONOUS"'
    assert setting in launch
    assert launch.index(setting) < launch.index("OpaqueFunction(function=launch_gazebo)")
    assert "RMW_FASTRTPS_USE_QOS_FROM_XML" not in launch


def test_factory_has_all_fixed_tag_ids_and_product_handle_geometry():
    world = (ROOT / "worlds" / "factory.sdf").read_text()
    assert 'name="gz::sim::systems::Contact"' in world
    for tag_id in (10, 11, 12, 20, 101, 102, 103):
        texture = ROOT / "models" / "phase14_tags" / "materials" / "textures" / (
            f"tag36h11_{tag_id}.png"
        )
        mesh = ROOT / "models" / "phase14_tags" / "meshes" / (
            f"tag36h11_{tag_id}.dae"
        )
        assert texture.is_file()
        assert mesh.is_file()
        assert f"tag36h11_{tag_id}.dae" in world
        assert f"tag36h11_{tag_id}.png" in mesh.read_text()
    sdf = ET.fromstring(world)
    products = {model.attrib["name"]: model for model in sdf.findall("./world/model")}
    for name, mass in (("product_a", 1.0), ("product_b", 3.0), ("product_c", 5.0)):
        product = products[name]
        assert float(product.find("./link/inertial/mass").text) == mass
        assert product.find("./link/collision[@name='body']/geometry/box/size").text == (
            "0.30 0.20 0.15"
        )
        assert product.find("./link/collision[@name='handle']/geometry/box/size").text == (
            "0.04 0.10 0.05"
        )
        publisher = product.find(
            "./plugin[@name='gz::sim::systems::PosePublisher']")
        assert publisher.find("publish_model_pose").text == "true"
        assert publisher.find("use_pose_vector_msg").text == "false"
