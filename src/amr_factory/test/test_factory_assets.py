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


def test_pickup_pedestal_map_cells_match_sdf_geometry():
    metadata = yaml.safe_load((ROOT / "maps" / "factory.yaml").read_text())
    resolution = metadata["resolution"]
    origin_x, origin_y, _ = metadata["origin"]
    occupied_cutoff = (1.0 - metadata["occupied_thresh"]) * 255.0

    with (ROOT / "maps" / "factory.pgm").open("rb") as image:
        assert image.readline() == b"P5\n"
        dimensions = image.readline().strip()
        while dimensions.startswith(b"#"):
            dimensions = image.readline().strip()
        width, height = map(int, dimensions.split())
        assert image.readline() == b"255\n"
        pixels = image.read()

    assert len(pixels) == width * height

    def grid_index(value, origin):
        index = (value - origin) / resolution
        assert math.isclose(index, round(index), abs_tol=1e-9)
        return int(round(index))

    def is_occupied(row, column):
        assert 0 <= row < height and 0 <= column < width
        return pixels[row * width + column] <= occupied_cutoff

    sdf = ET.fromstring((ROOT / "worlds" / "factory.sdf").read_text())
    for station_name, expected_rows in (
        ("pickup_a", range(35, 45)),
        ("pickup_b", range(95, 105)),
        ("pickup_c", range(155, 165)),
    ):
        collision = sdf.find(
            f".//collision[@name='{station_name}_pedestal']")
        pose = [float(value) for value in collision.find("pose").text.split()]
        size = [float(value) for value in collision.find(
            "geometry/box/size").text.split()]
        min_column = grid_index(pose[0] - size[0] / 2.0, origin_x)
        max_column = grid_index(pose[0] + size[0] / 2.0, origin_x)
        min_map_row = grid_index(pose[1] - size[1] / 2.0, origin_y)
        max_map_row = grid_index(pose[1] + size[1] / 2.0, origin_y)
        image_rows = range(height - max_map_row, height - min_map_row)
        image_columns = range(min_column, max_column)

        assert tuple(image_rows) == tuple(expected_rows)
        assert tuple(image_columns) == tuple(range(182, 190))
        expected_occupied = {
            (row, column)
            for row in image_rows
            for column in image_columns
        }
        local_window_rows = range(image_rows.start - 1, image_rows.stop)
        local_window_columns = range(image_columns.start, image_columns.stop + 2)
        observed_occupied = {
            (row, column)
            for row in local_window_rows
            for column in local_window_columns
            if is_occupied(row, column)
        }
        assert observed_occupied == expected_occupied, (
            f"{station_name} pedestal occupancy differs from its SDF-derived "
            f"cell block: unexpected={observed_occupied - expected_occupied}, "
            f"missing={expected_occupied - observed_occupied}"
        )


def test_factory_right_side_map_cells_match_sdf_geometry():
    metadata = yaml.safe_load((ROOT / "maps" / "factory.yaml").read_text())
    resolution = metadata["resolution"]
    origin_x, origin_y, _ = metadata["origin"]
    occupied_cutoff = (1.0 - metadata["occupied_thresh"]) * 255.0

    with (ROOT / "maps" / "factory.pgm").open("rb") as image:
        assert image.readline() == b"P5\n"
        width, height = map(int, image.readline().split())
        assert image.readline() == b"255\n"
        pixels = image.read()
    assert len(pixels) == width * height

    def lower_cell_index(value, origin):
        return math.floor((value - origin) / resolution + 1e-9)

    def upper_cell_index(value, origin):
        return math.ceil((value - origin) / resolution - 1e-9)

    def cells_for_bounds(bounds):
        min_x, max_x, min_y, max_y = bounds
        min_column = max(0, lower_cell_index(min_x, origin_x))
        max_column = min(width, upper_cell_index(max_x, origin_x))
        min_map_row = max(0, lower_cell_index(min_y, origin_y))
        max_map_row = min(height, upper_cell_index(max_y, origin_y))
        return {
            (row, column)
            for row in range(height - max_map_row, height - min_map_row)
            for column in range(min_column, max_column)
        }

    def is_occupied(row, column):
        return pixels[row * width + column] <= occupied_cutoff

    def sdf_box_bounds(collision):
        pose = [float(value) for value in collision.find("pose").text.split()]
        size = [float(value) for value in collision.find(
            "geometry/box/size").text.split()]
        return (
            pose[0] - size[0] / 2.0,
            pose[0] + size[0] / 2.0,
            pose[1] - size[1] / 2.0,
            pose[1] + size[1] / 2.0,
        )

    sdf = ET.fromstring((ROOT / "worlds" / "factory.sdf").read_text())
    world = sdf.find("world")
    expected_occupied = set()
    for boundary_name in ("east", "north", "south"):
        collision = world.find(
            f"./model[@name='factory_boundary']/link/collision[@name='{boundary_name}']"
        )
        expected_occupied.update(cells_for_bounds(sdf_box_bounds(collision)))

    dae_namespace = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
    dae = ET.fromstring((ROOT / "models" /
                         "aws_robomaker_warehouse_ShelfD_01" /
                         "meshes/aws_robomaker_warehouse_ShelfD_01_collision.DAE"
                         ).read_text())
    unit = float(dae.find("./c:asset/c:unit", dae_namespace).attrib["meter"])
    matrix = [float(value) for value in dae.find(
        ".//c:visual_scene/c:node/c:matrix", dae_namespace).text.split()]
    position_source = next(
        source for source in dae.findall(".//c:source", dae_namespace)
        if source.attrib["id"].endswith("-POSITION")
    )
    raw_values = [float(value) for value in position_source.find(
        "c:float_array", dae_namespace).text.split()]
    raw_points = list(zip(raw_values[0::3], raw_values[1::3], raw_values[2::3]))

    def dae_transform(point):
        point_m = [value * unit for value in point]
        return tuple(
            sum(matrix[row * 4 + column] * point_m[column]
                for column in range(3))
            + matrix[row * 4 + 3] * unit
            for row in range(3)
        )

    triangles = dae.find(".//c:triangles", dae_namespace)
    inputs = triangles.findall("c:input", dae_namespace)
    vertex_input = next(
        item for item in inputs if item.attrib["semantic"] == "VERTEX"
    )
    vertex_offset = int(vertex_input.attrib["offset"])
    stride = max(int(item.attrib["offset"]) for item in inputs) + 1
    indices = [int(value) for value in triangles.find(
        "c:p", dae_namespace).text.split()]
    vertex_indices = indices[vertex_offset::stride]
    assert len(vertex_indices) % 3 == 0

    adjacency = {index: set() for index in range(len(raw_points))}
    for first, second, third in zip(
        vertex_indices[0::3], vertex_indices[1::3], vertex_indices[2::3]
    ):
        for left, right in ((first, second), (second, third), (third, first)):
            adjacency[left].add(right)
            adjacency[right].add(left)
    components = []
    unseen = set(adjacency)
    while unseen:
        pending = [unseen.pop()]
        component = set(pending)
        while pending:
            for neighbor in adjacency[pending.pop()] & unseen:
                unseen.remove(neighbor)
                component.add(neighbor)
                pending.append(neighbor)
        components.append(component)

    transformed_components = [
        [dae_transform(raw_points[index]) for index in component]
        for component in components
    ]
    lidar_height = 0.52248
    scan_components = [
        component for component in transformed_components
        if min(point[2] for point in component) <= lidar_height <= max(
            point[2] for point in component
        )
    ]
    assert len(scan_components) == 3

    expected_scan_rows = {
        "pickup_a": {2, 3, 41, 42, 80, 81},
        "pickup_b": {60, 61, 99, 100, 138, 139},
        "pickup_c": {118, 119, 157, 158, 196, 197},
    }
    for station_name in ("pickup_a", "pickup_b", "pickup_c"):
        include = next(
            include for include in world.findall("./include")
            if include.findtext("name") == f"{station_name}_shelf"
        )
        pose = [float(value) for value in include.find("pose").text.split()]
        cosine = math.cos(pose[5])
        sine = math.sin(pose[5])
        station_scan_cells = set()
        for component in scan_components:
            world_points = [
                (
                    pose[0] + cosine * point[0] - sine * point[1],
                    pose[1] + sine * point[0] + cosine * point[1],
                )
                for point in component
            ]
            bounds = (
                min(point[0] for point in world_points),
                max(point[0] for point in world_points),
                min(point[1] for point in world_points),
                max(point[1] for point in world_points),
            )
            station_scan_cells.update(cells_for_bounds(bounds))
        assert {row for row, _ in station_scan_cells} == expected_scan_rows[
            station_name
        ]
        assert {column for _, column in station_scan_cells} == set(
            range(218, 236)
        )
        expected_occupied.update(station_scan_cells)

    local_window_start = lower_cell_index(4.0, origin_x)
    assert local_window_start == 200
    local_window = {
        (row, column)
        for row in range(height)
        for column in range(local_window_start, width)
    }
    expected_local_occupied = expected_occupied & local_window
    observed_local_occupied = {
        (row, column)
        for row, column in local_window
        if is_occupied(row, column)
    }
    assert observed_local_occupied == expected_local_occupied, (
        "right-side map occupancy differs from the transformed SDF/DAE "
        "geometry: "
        f"unexpected={observed_local_occupied - expected_local_occupied}, "
        f"missing={expected_local_occupied - observed_local_occupied}"
    )


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


def test_factory_amcl_motion_noise_is_zero_for_deterministic_simulation():
    config = yaml.safe_load((ROOT / "config" / "amcl.yaml").read_text())
    parameters = config["/amr/amcl"]["ros__parameters"]
    assert {
        key: parameters[key]
        for key in ("alpha1", "alpha2", "alpha3", "alpha4", "alpha5")
    } == {
        "alpha1": 0.0,
        "alpha2": 0.0,
        "alpha3": 0.0,
        "alpha4": 0.0,
        "alpha5": 0.0,
    }


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


def test_fastdds_service_profile_is_the_only_one_second_publisher_override():
    profile_path = ROOT / "config" / "fastdds_service_profiles.xml"
    root = ET.parse(profile_path).getroot()
    namespace = "http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"

    def qualified(tag):
        return f"{{{namespace}}}{tag}"

    assert root.tag == qualified("profiles")
    assert root.attrib == {}
    assert len(root) == 1
    publishers = root.findall(qualified("publisher"))
    assert len(publishers) == 1
    publisher = publishers[0]
    assert publisher.attrib == {"profile_name": "service"}
    assert [child.tag for child in publisher] == [qualified("qos")]

    qos = publisher.find(qualified("qos"))
    assert [child.tag for child in qos] == [qualified("reliability")]
    reliability = qos.find(qualified("reliability"))
    assert [child.tag for child in reliability] == [
        qualified("max_blocking_time")
    ]
    max_blocking_time = reliability.find(qualified("max_blocking_time"))
    assert [child.tag for child in max_blocking_time] == [qualified("sec")]
    assert max_blocking_time.findtext(qualified("sec")) == "1"


def test_factory_launch_installs_fastdds_service_profile_before_children():
    launch = (ROOT / "launch" / "factory_localization.launch.py").read_text()
    path_assignment_start = launch.index(
        "fastdds_profile_path = os.path.abspath(os.path.join("
    )
    path_assignment_end = launch.index(
        'factory, "config", "fastdds_service_profiles.xml"))',
        path_assignment_start,
    )
    setting = (
        'name="FASTRTPS_DEFAULT_PROFILES_FILE", value=fastdds_profile_path'
    )

    assert 'factory = get_package_share_directory("amr_factory")' in launch
    assert path_assignment_start < path_assignment_end
    assert setting in launch
    assert "FASTDDS_DEFAULT_PROFILES_FILE" not in launch
    profile_setting_start = launch.index(setting)
    assert path_assignment_end < profile_setting_start
    assert profile_setting_start < launch.index(
        "OpaqueFunction(function=launch_gazebo)"
    )
    assert profile_setting_start < launch.index("bridge,", profile_setting_start)
    assert profile_setting_start < launch.index(
        "OpaqueFunction(function=launch_robot)", profile_setting_start
    )


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
