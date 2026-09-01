import hashlib
import math
import struct
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml
import pytest
from ament_index_python.packages import get_package_prefix

ROOT = Path(__file__).resolve().parents[1]


def stl_vertices(path):
    data = path.read_bytes()
    triangle_count = struct.unpack_from("<I", data, 80)[0]
    assert len(data) == 84 + 50 * triangle_count
    for offset in range(84, len(data), 50):
        values = struct.unpack_from("<12fH", data, offset)
        yield from (values[3:6], values[6:9], values[9:12])

def expand(*args):
    return ET.fromstring(subprocess.check_output(["xacro", str(ROOT / "urdf" / "amr.urdf.xacro"), *args], text=True))

def test_default_model_has_expected_frames_and_positive_inertias():
    robot = expand()
    links = {link.attrib["name"] for link in robot.findall("link")}
    assert {"base_footprint", "base_link", "payload_link", "imu_link", "front_lidar_link", "rear_lidar_link"} <= links
    assert len([link for link in links if "caster" in link]) == 8
    for inertia in robot.findall(".//inertia"):
        assert all(math.isfinite(float(inertia.attrib[key])) and float(inertia.attrib[key]) > 0 for key in ("ixx", "iyy", "izz"))


def test_base_representation_uses_cad_visual_and_preserves_navigation_footprint():
    robot = expand("include_generic_payload:=false")
    base = robot.find("./link[@name='base_link']")
    visuals = base.findall("./visual")
    assert len(visuals) == 1
    assert visuals[0].find("./geometry/mesh").attrib == {
        "filename": "package://amr_description/meshes/base_link.STL"}
    assert visuals[0].find("./origin").attrib == {
        "xyz": "0 0 0", "rpy": "0 0 0"}

    expected_meshes = {
        "base_link.STL",
        "left_wheel_link.STL", "right_wheel_link.STL",
        "lidar_front_link.STL", "lidar_back_link.STL",
        "left_caster_front_link_body.STL", "left_caster_front_link_wheel.STL",
        "right_caster_front_link_body.STL", "right_caster_front_link_wheel.STL",
        "left_caster_back_link_body.STL", "left_caster_back_link_wheel.STL",
        "right_caster_back_link_body.STL", "right_caster_back_link_wheel.STL",
    }
    mesh_uris = [mesh.attrib["filename"]
                 for mesh in robot.findall(".//visual/geometry/mesh")]
    assert len(mesh_uris) == len(expected_meshes) == 13
    assert {uri.rsplit("/", 1)[-1] for uri in mesh_uris} == expected_meshes
    assert all(uri.startswith("package://amr_description/meshes/") for uri in mesh_uris)
    assert "amr_urdf_cad" not in ET.tostring(robot, encoding="unicode")

    for geometry in robot.findall(".//collision/geometry"):
        assert geometry.find("mesh") is None
        assert geometry.find("box") is not None or geometry.find("cylinder") is not None

    chassis = base.find("./collision[@name='lower_chassis_collision']/geometry/box")
    assert tuple(map(float, chassis.attrib["size"].split())) == (1.20, 0.80, 0.35)
    lower = base.find("./collision[@name='lower_chassis_collision']")
    assert lower.find("origin").attrib["xyz"] == "0 0 0.155"
    assert tuple(map(float, lower.find("geometry/box").attrib["size"].split())) == (
        1.20, 0.80, 0.35)
    assert base.find("./collision[@name='pedestal_proxy_collision']") is None
    assert base.find("./visual[@name='pedestal_proxy_visual']") is None
    assert not any(
        token in element.attrib.get("name", "").lower()
        for element in (*base.findall("./visual"), *base.findall("./collision"))
        for token in ("pedestal", "plate")
    )
    chassis_top_z = float(lower.find("origin").attrib["xyz"].split()[2]) + float(
        lower.find("geometry/box").attrib["size"].split()[2]) / 2.0
    chassis_bottom_z = float(lower.find("origin").attrib["xyz"].split()[2]) - float(
        lower.find("geometry/box").attrib["size"].split()[2]) / 2.0
    assert chassis_bottom_z == pytest.approx(-0.02, abs=1e-12)
    assert chassis_top_z == pytest.approx(0.33, abs=1e-12)
    base_mass = float(base.find("./inertial/mass").attrib["value"])
    assert base_mass == pytest.approx(22.15)
    base_height = robot.find("./joint[@name='base_footprint_joint']/origin")
    assert base_height.attrib["xyz"] == "0 0 0.0478"

    default = expand()
    generic_payload_support_top_z = 0.54263913635
    payload_joint = default.find("./joint[@name='payload_joint']/origin")
    payload_size = float(default.find(
        "./link[@name='payload_link']/visual/geometry/box").attrib["size"].split()[2])
    assert float(payload_joint.attrib["xyz"].split()[2]) - payload_size / 2.0 == pytest.approx(
        generic_payload_support_top_z + 0.01, abs=1e-12)


def test_derived_base_mesh_is_hash_size_triangle_and_height_gated():
    source = ROOT.parents[1] / "amr_urdf_cad" / "meshes" / "base_link.STL"
    derived = ROOT / "meshes" / "base_link.STL"
    source_bytes = source.read_bytes()
    derived_bytes = derived.read_bytes()
    assert hashlib.sha256(source_bytes).hexdigest() == (
        "b0f27db25987905634d2bff27f52c68fd1fe90f1ce1d0977ec57de63eb44d015")
    assert hashlib.sha256(derived_bytes).hexdigest() == (
        "9fc86036c41e1e846b931de430371f7f19f19c913824f6124dfaeec8b8c21336")
    assert len(derived_bytes) == 4_808_984
    assert struct.unpack_from("<I", derived_bytes, 80)[0] == 96_178
    max_z = max(vertex[2] for vertex in stl_vertices(derived))
    assert max_z == pytest.approx(0.33, abs=1e-7)


def test_cad_visual_materials_are_explicit_nonblack_and_use_legacy_colors():
    robot = expand()
    legacy_body = (0.752941176470588, 0.752941176470588,
                   0.752941176470588, 1.0)
    legacy_bluegray = (0.792156862745098, 0.819607843137255,
                       0.933333333333333, 1.0)
    for visual in robot.findall(".//visual"):
        material = visual.find("./material")
        assert material is not None
        assert not material.attrib.get("name", "").startswith("Gazebo/")
        color = material.find("./color")
        assert color is not None
        rgba = tuple(map(float, color.attrib["rgba"].split()))
        assert len(rgba) == 4
        assert any(channel > 0.0 for channel in rgba[:3])

    cad_colors = {}
    for visual in robot.findall(".//visual"):
        mesh = visual.find("./geometry/mesh")
        if mesh is not None:
            name = mesh.attrib["filename"].rsplit("/", 1)[-1]
            cad_colors[name] = tuple(map(float, visual.find(
                "./material/color").attrib["rgba"].split()))
    assert cad_colors["base_link.STL"] == legacy_body
    assert cad_colors["left_wheel_link.STL"] == legacy_bluegray
    assert cad_colors["right_wheel_link.STL"] == legacy_bluegray
    assert cad_colors["lidar_front_link.STL"] == legacy_bluegray
    assert cad_colors["lidar_back_link.STL"] == legacy_bluegray
    assert all(cad_colors[name] == legacy_body for name in cad_colors
               if "caster" in name)


def test_installed_cad_meshes_resolve_and_match_source_bytes():
    robot = expand("include_generic_payload:=false")
    package_prefix = Path(get_package_prefix("amr_description"))
    installed_mesh_dir = package_prefix / "share" / "amr_description" / "meshes"
    for mesh in robot.findall(".//visual/geometry/mesh"):
        uri = mesh.attrib["filename"]
        package, relative = uri.removeprefix("package://").split("/", 1)
        assert package == "amr_description"
        source = ROOT / relative
        installed = installed_mesh_dir / Path(relative).name
        assert source.is_file()
        assert installed.is_file()
        assert installed.read_bytes() == source.read_bytes()


def test_drive_and_caster_contacts_share_the_ground_plane():
    robot = expand()
    base_height = float(robot.find("./joint[@name='base_footprint_joint']/origin").attrib["xyz"].split()[2])
    drive_radius = 0.1128
    caster_radius = 0.0393
    for name in ("left_wheel", "right_wheel"):
        joint = robot.find(f"./joint[@name='{name}_joint']/origin")
        collision = robot.find(f"./link[@name='{name}']/collision/geometry/cylinder")
        assert float(collision.attrib["radius"]) == drive_radius
        assert float(collision.attrib["length"]) == 0.16
        wheel_center_z = float(joint.attrib["xyz"].split()[2]) + base_height
        assert wheel_center_z - drive_radius == pytest.approx(0.0, abs=1e-12)
    caster_mounts = {
        "front_left_caster": (0.32, 0.165),
        "front_right_caster": (0.32, -0.165),
        "rear_left_caster": (-0.32, 0.165),
        "rear_right_caster": (-0.32, -0.165),
    }
    for name, (mount_x, mount_y) in caster_mounts.items():
        swivel = robot.find(f"./joint[@name='{name}_swivel_joint']/origin")
        rolling = robot.find(f"./joint[@name='{name}_wheel_joint']")
        collision = robot.find(f"./link[@name='{name}_wheel']/collision/geometry/cylinder")
        assert collision.attrib == {"radius": "0.0393", "length": "0.0421"}
        assert tuple(map(float, swivel.attrib["xyz"].split())) == pytest.approx(
            (mount_x, mount_y, -0.0085))
        assert float(swivel.attrib["xyz"].split()[2]) + base_height - caster_radius == pytest.approx(0.0, abs=1e-12)
        assert rolling.attrib["type"] == "continuous"


def test_generic_payload_modes_are_base_only_default_on_and_composite_default_off():
    assert expand().find("./link[@name='payload_link']") is not None
    assert expand("include_generic_payload:=false").find("./link[@name='payload_link']") is None
    assert expand_composite().find("./link[@name='payload_link']") is None
    assert ET.fromstring(subprocess.check_output([
        "xacro", str(ROOT / "urdf" / "phase14_mobile_manipulator.urdf.xacro"),
        "include_generic_payload:=true",
    ], text=True)).find("./link[@name='payload_link']") is not None

def test_default_mass_is_approximately_eighty_kg_and_payload_is_overridable():
    default = expand()
    override = expand("payload_mass:=20.0")
    default_mass = sum(float(item.attrib["value"]) for item in default.findall(".//mass"))
    override_mass = sum(float(item.attrib["value"]) for item in override.findall(".//mass"))
    assert 79.0 <= default_mass <= 83.0
    assert math.isclose(default_mass - override_mass, 30.0, abs_tol=1e-6)
    default_ixx = float(default.find("./link[@name='payload_link']/inertial/inertia").attrib["ixx"])
    override_ixx = float(override.find("./link[@name='payload_link']/inertial/inertia").attrib["ixx"])
    assert math.isclose(override_ixx / default_ixx, 20.0 / 50.0)

def test_invalid_payload_parameters_are_rejected():
    for argument in ("payload_mass:=0", "payload_mass:=-1", "payload_x:=0",
                     "payload_y:=-1", "payload_z:=nan"):
        result = subprocess.run(
            ["xacro", str(ROOT / "urdf" / "amr.urdf.xacro"), argument],
            capture_output=True, check=False)
        assert result.returncode != 0

def test_cad_visual_transforms_preserve_joint_semantics():
    robot = expand()
    drive_meshes = {
        "left_wheel": "left_wheel_link.STL",
        "right_wheel": "right_wheel_link.STL",
    }
    drive_joints = {
        "left_wheel": {
            "xyz": "0 0.283 0.065", "rpy": "-1.57079632679 0 0",
            "axis": (0.0, 0.0, 1.0)},
        "right_wheel": {
            "xyz": "0 -0.283 0.065", "rpy": "1.57079632679 0 0",
            "axis": (0.0, 0.0, -1.0)},
    }
    for name, mesh_name in drive_meshes.items():
        visual = robot.find(f"./link[@name='{name}']/visual")
        assert visual.find("./geometry/mesh").attrib["filename"].endswith(mesh_name)
        assert visual.find("./origin").attrib == {"xyz": "0 0 0", "rpy": "0 0 0"}
        joint = robot.find(f"./joint[@name='{name}_joint']")
        expected = drive_joints[name]
        assert joint.find("./origin").attrib == {
            "xyz": expected["xyz"], "rpy": expected["rpy"]}
        assert tuple(map(float, joint.find("./axis").attrib["xyz"].split())) == expected["axis"]
        roll, pitch, yaw = map(float, expected["rpy"].split())
        x, y, z = expected["axis"]
        cy, sy = math.cos(pitch), math.sin(pitch)
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(yaw), math.sin(yaw)
        transformed_axis = (
            cy * cp * x + (cy * sp * sr - sy * cr) * y +
            (cy * sp * cr + sy * sr) * z,
            sy * cp * x + (sy * sp * sr + cy * cr) * y +
            (sy * sp * cr - cy * sr) * z,
            -sp * x + cp * sr * y + cp * cr * z)
        assert transformed_axis == pytest.approx((0.0, 1.0, 0.0), abs=1e-10)

    lidar_meshes = {
        "front_lidar_link": "lidar_front_link.STL",
        "rear_lidar_link": "lidar_back_link.STL",
    }
    for name, mesh_name in lidar_meshes.items():
        visual = robot.find(f"./link[@name='{name}']/visual")
        assert visual.find("./geometry/mesh").attrib["filename"].endswith(mesh_name)
        assert visual.find("./origin").attrib == {"xyz": "0 0 0", "rpy": "0 0 0"}

    casters = {
        "front_left_caster": (
            "left_caster_front_link_body.STL", 0.0568,
            "left_caster_front_link_wheel.STL", -0.025, 0.0000775,
        ),
        "front_right_caster": (
            "right_caster_front_link_body.STL", 0.065,
            "right_caster_front_link_wheel.STL", -0.025, -0.0000775,
        ),
        "rear_left_caster": (
            "left_caster_back_link_body.STL", 0.0568,
            "left_caster_back_link_wheel.STL", 0.025, 0.0000775,
        ),
        "rear_right_caster": (
            "right_caster_back_link_body.STL", 0.0568,
            "right_caster_back_link_wheel.STL", 0.025, -0.0000775,
        ),
    }
    for name, (body_mesh, body_z, wheel_mesh, wheel_x, wheel_y) in casters.items():
        body_visual = robot.find(f"./link[@name='{name}_mount']/visual")
        assert body_visual.find("./geometry/mesh").attrib["filename"].endswith(body_mesh)
        assert float(body_visual.find("./origin").attrib["xyz"].split()[2]) == pytest.approx(body_z)
        wheel_visual = robot.find(f"./link[@name='{name}_wheel']/visual")
        assert wheel_visual.find("./geometry/mesh").attrib["filename"].endswith(wheel_mesh)
        assert wheel_visual.find("./origin").attrib["rpy"] == "-1.57079632679 0 0"
        rolling = robot.find(f"./joint[@name='{name}_wheel_joint']")
        rolling_origin = rolling.find("./origin")
        assert tuple(map(float, rolling_origin.attrib["xyz"].split())) == pytest.approx(
            (wheel_x, wheel_y, 0.0))
        assert rolling_origin.attrib["rpy"] == "1.57079632679 0 0"
        assert rolling.find("./axis").attrib["xyz"] == "0 0 1"

def test_passive_caster_contacts_do_not_lock_nominal_turning():
    robot = expand()
    for name in (
        "front_left_caster_wheel",
        "front_right_caster_wheel",
        "rear_left_caster_wheel",
        "rear_right_caster_wheel",
    ):
        gazebo = robot.find(f"./gazebo[@reference='{name}']")
        assert float(gazebo.find("mu1").text) == 0.01
        assert float(gazebo.find("mu2").text) == 0.01

def test_lidars_are_on_opposite_corners_facing_outward():
    robot = expand()
    front = robot.find("./joint[@name='front_lidar_joint']/origin")
    rear = robot.find("./joint[@name='rear_lidar_joint']/origin")
    assert front.attrib == {"xyz": "0.30805 -0.153 0.47468", "rpy": "0 0 0"}
    assert rear.attrib == {"xyz": "-0.308 0.153 0.47468", "rpy": "0 0 -3.1416"}

def test_lidars_are_outside_each_others_field_of_view():
    robot = expand()
    poses = {}
    for name in ("front_lidar_joint", "rear_lidar_joint"):
        origin = robot.find(f"./joint[@name='{name}']/origin")
        x, y, _ = map(float, origin.attrib["xyz"].split())
        yaw = float(origin.attrib["rpy"].split()[2])
        poses[name] = (x, y, yaw)
    for source, target in (("front_lidar_joint", "rear_lidar_joint"),
                           ("rear_lidar_joint", "front_lidar_joint")):
        source_x, source_y, source_yaw = poses[source]
        target_x, target_y, _ = poses[target]
        bearing = math.atan2(target_y - source_y, target_x - source_x)
        relative_bearing = math.atan2(
            math.sin(bearing - source_yaw), math.cos(bearing - source_yaw))
        assert abs(relative_bearing) > 2.4


def test_lidar_scan_rays_do_not_intersect_the_chassis():
    robot = expand()
    chassis_size = tuple(map(float, robot.find(
        "./link[@name='base_link']/collision/geometry/box").attrib["size"].split()))
    bounds = tuple((-size / 2.0, size / 2.0) for size in chassis_size)

    def samples(scan, axis):
        definition = scan.find(axis)
        count = int(definition.find("samples").text)
        lower = float(definition.find("min_angle").text)
        upper = float(definition.find("max_angle").text)
        assert count > 1
        return [lower + index * (upper - lower) / (count - 1)
                for index in range(count)]

    def intersects_chassis(origin, direction, minimum_range, maximum_range):
        entry = minimum_range
        exit = maximum_range
        for coordinate, component, (lower, upper) in zip(
                origin, direction, bounds):
            if math.isclose(component, 0.0, abs_tol=1e-12):
                if coordinate < lower or coordinate > upper:
                    return False
                continue
            first = (lower - coordinate) / component
            second = (upper - coordinate) / component
            entry = max(entry, min(first, second))
            exit = min(exit, max(first, second))
            if entry > exit:
                return False
        return exit >= entry

    for prefix in ("front", "rear"):
        joint_origin = robot.find(
            f"./joint[@name='{prefix}_lidar_joint']/origin")
        origin = tuple(map(float, joint_origin.attrib["xyz"].split()))
        yaw = float(joint_origin.attrib["rpy"].split()[2])
        sensor = robot.find(
            f"./gazebo[@reference='{prefix}_lidar_link']"
            f"/sensor[@name='{prefix}_lidar']")
        scan = sensor.find("lidar/scan")
        ranges = sensor.find("lidar/range")
        minimum_range = float(ranges.find("min").text)
        maximum_range = float(ranges.find("max").text)
        horizontal_angles = samples(scan, "horizontal")
        vertical_angles = samples(scan, "vertical")
        assert len(horizontal_angles) == 720
        assert len(vertical_angles) == 4

        for vertical in vertical_angles:
            for horizontal in horizontal_angles:
                direction = (
                    math.cos(vertical) * math.cos(yaw + horizontal),
                    math.cos(vertical) * math.sin(yaw + horizontal),
                    math.sin(vertical),
                )
                assert not intersects_chassis(
                    origin, direction, minimum_range, maximum_range)

def test_urdf_converts_to_valid_sdf():
    urdf = subprocess.check_output(["xacro", str(ROOT / "urdf" / "amr.urdf.xacro")])
    with tempfile.NamedTemporaryFile(suffix=".urdf") as source:
        source.write(urdf)
        source.flush()
        sdf = subprocess.check_output(["gz", "sdf", "-p", source.name])
    root = ET.fromstring(sdf)
    assert root.tag == "sdf"
    base_visual = next(
        (visual for visual in root.findall(".//visual")
         if visual.find("./geometry/mesh/uri") is not None
         and visual.find("./geometry/mesh/uri").text.endswith("/base_link.STL")),
        None,
    )
    base_diffuse = None if base_visual is None else base_visual.find("./material/diffuse")
    assert base_diffuse is not None
    assert any(float(channel) > 0.0 for channel in base_diffuse.text.split()[:3])

def test_diff_drive_uses_commissioning_limits():
    robot = expand()
    plugin = robot.find("./gazebo/plugin[@name='gz::sim::systems::DiffDrive']")
    assert float(plugin.find("max_linear_velocity").text) == 0.5
    assert float(plugin.find("min_linear_velocity").text) == -0.5
    assert float(plugin.find("max_angular_velocity").text) == 0.4
    assert float(plugin.find("min_angular_velocity").text) == -0.4
    assert float(plugin.find("max_linear_acceleration").text) == 0.5
    assert float(plugin.find("max_angular_acceleration").text) == 0.4
    assert float(plugin.find("odom_publish_frequency").text) == 50.0

def test_model_pose_is_available_as_independent_acceptance_truth():
    robot = expand()
    plugin = robot.find("./gazebo/plugin[@name='gz::sim::systems::PosePublisher']")
    assert plugin.find("publish_model_pose").text == "true"
    assert plugin.find("publish_link_pose").text == "false"
    assert plugin.find("use_pose_vector_msg").text == "false"


def test_default_joint_state_topic_remains_the_base_only_world():
    robot = expand()
    plugin = robot.find(
        "./gazebo/plugin[@name='gz::sim::systems::JointStatePublisher']")
    assert plugin.find("topic").text == "/world/amr_world/model/amr/joint_state"

def test_sensor_frame_ids_match_urdf_links():
    robot = expand()
    for link_name, sensor_name in (
        ("front_lidar_link", "front_lidar"),
        ("rear_lidar_link", "rear_lidar"),
        ("imu_link", "imu"),
    ):
        sensor = robot.find(
            f"./gazebo[@reference='{link_name}']/sensor[@name='{sensor_name}']")
        assert sensor.find("gz_frame_id").text == link_name


def test_phase14_standalone_kuka_has_prefixed_harmonic_control_contract():
    robot = ET.fromstring(subprocess.check_output(
        [
            "xacro",
            str(ROOT / "urdf" / "phase14_kr6_r900_2.urdf.xacro"),
            f"controller_config:={ROOT / 'config' / 'phase14_arm_controllers.yaml'}",
        ],
        text=True,
    ))
    arm_joints = [f"arm_joint_{index}" for index in range(1, 7)]
    assert {f"arm_link_{index}" for index in range(1, 7)} <= {
        link.attrib["name"] for link in robot.findall("link")
    }
    control = robot.find("./ros2_control[@name='Phase14KukaSystem']")
    assert control.find("./hardware/plugin").text == "gz_ros2_control/GazeboSimSystem"
    assert [joint.attrib["name"] for joint in control.findall("joint")] == arm_joints
    assert all(
        joint.find("./command_interface[@name='position']") is not None
        for joint in control.findall("joint")
    )
    plugin = robot.find("./gazebo/plugin[@name='gz_ros2_control::GazeboSimROS2ControlPlugin']")
    assert plugin.attrib["filename"] == "gz_ros2_control-system"

    controllers = yaml.safe_load(
        (ROOT / "config" / "phase14_arm_controllers.yaml").read_text())
    assert controllers["controller_manager"]["ros__parameters"]["arm_controller"]["type"] == (
        "joint_trajectory_controller/JointTrajectoryController")
    assert controllers["arm_controller"]["ros__parameters"]["joints"] == arm_joints


def test_composite_controller_uses_strict_tracking_tolerances():
    controllers = yaml.safe_load(
        (ROOT / "config" / "phase14_mobile_manipulator_controllers.yaml").read_text())
    manager = controllers["controller_manager"]["ros__parameters"]
    assert manager["update_rate"] == 100
    assert "position_proportional_gain" not in manager
    assert controllers["gz_ros2_control"]["ros__parameters"] == {
        "position_proportional_gain": 1.0,
    }

    constraints = controllers["arm_controller"]["ros__parameters"]["constraints"]
    assert constraints["goal_time"] == 1.0
    assert constraints["stopped_velocity_tolerance"] == 0.01
    for joint in [f"arm_joint_{index}" for index in range(1, 7)]:
        assert constraints[joint] == {"trajectory": 0.05, "goal": 0.01}


def expand_composite(loaded=False):
    return ET.fromstring(subprocess.check_output(
        [
            "xacro",
            str(ROOT / "urdf" / "phase14_mobile_manipulator.urdf.xacro"),
            f"controller_config:={ROOT / 'config' / 'phase14_mobile_manipulator_controllers.yaml'}",
            f"loaded_product:={'true' if loaded else 'false'}",
        ],
        text=True,
    ))


def expand_factory_composite():
    return ET.fromstring(subprocess.check_output(
        [
            "xacro",
            str(ROOT / "urdf" / "phase14_mobile_manipulator.urdf.xacro"),
            f"controller_config:={ROOT / 'config' / 'phase14_mobile_manipulator_controllers.yaml'}",
            "factory_attachment:=true",
        ],
        text=True,
    ))


def test_phase14_composite_has_unique_names_fixed_mount_and_mass_budget():
    robot = expand_composite()
    links = [link.attrib["name"] for link in robot.findall("link")]
    joints = [joint.attrib["name"] for joint in robot.findall("joint")]
    assert len(links) == len(set(links))
    assert len(joints) == len(set(joints))
    assert "payload_link" not in links
    assert "arm_pedestal_link" not in links
    assert "arm_pedestal_joint" not in joints
    assert not any("plate" in name for name in links)
    assert not any("plate" in name for name in joints)
    assert {"base_link", "arm_base_link", "gripper_tcp",
            "product_camera_optical_frame"} <= set(links)

    mount = robot.find("./joint[@name='arm_mount_joint']/origin")
    mount_joint = robot.find("./joint[@name='arm_mount_joint']")
    assert mount_joint.find("parent").attrib["link"] == "base_link"
    assert mount_joint.find("child").attrib["link"] == "arm_base_link"
    assert mount.attrib["rpy"] == "0 0 0"
    assert mount.attrib["xyz"] == "0 0 0.33"
    base = robot.find("./link[@name='base_link']")
    assert base.find("./visual[@name='pedestal_proxy_visual']") is None
    assert base.find("./collision[@name='pedestal_proxy_collision']") is None
    assert not any("pedestal" in name for name in links)
    assert not any("pedestal" in name for name in joints)
    lower = base.find("./collision[@name='lower_chassis_collision']")
    chassis_top_z = float(lower.find("origin").attrib["xyz"].split()[2]) + float(
        lower.find("geometry/box").attrib["size"].split()[2]) / 2.0
    chassis_bottom_z = float(lower.find("origin").attrib["xyz"].split()[2]) - float(
        lower.find("geometry/box").attrib["size"].split()[2]) / 2.0
    mount_z = float(mount.attrib["xyz"].split()[2])
    # The unchanged upstream KUKA collision mesh underhang is intentionally
    # covered by the existing narrow base_link <-> arm_base_link adjacency.
    kuka_base_underhang = -0.002757532522082329
    assert chassis_bottom_z == pytest.approx(-0.02, abs=1e-12)
    assert mount_z == pytest.approx(chassis_top_z, abs=1e-12)
    kuka_base_bottom_z = mount_z + kuka_base_underhang
    assert kuka_base_bottom_z == pytest.approx(0.32724246747791767, abs=1e-12)
    assert chassis_top_z - kuka_base_bottom_z == pytest.approx(
        -kuka_base_underhang, abs=1e-12)

    total_mass = sum(float(mass.attrib["value"]) for mass in robot.findall(".//mass"))
    gripper_mass = sum(float(robot.find(f"./link[@name='{name}']/inertial/mass").attrib["value"])
                       for name in ("gripper_base_link", "gripper_left_finger_link",
                                    "gripper_right_finger_link"))
    assert 80.0 <= total_mass <= 90.0
    assert math.isclose(gripper_mass, 0.8, abs_tol=1e-8)

    arm_joints = [joint for joint in robot.findall("joint")
                  if joint.attrib["name"].startswith("arm_joint_")]
    assert len(arm_joints) == 6
    assert {joint.attrib["name"] for joint in arm_joints} == {
        f"arm_joint_{index}" for index in range(1, 7)}
    assert all(joint.attrib["type"] in {"revolute", "continuous"}
               for joint in arm_joints)

    camera_joint = robot.find("./joint[@name='product_camera_joint']/origin")
    assert camera_joint.attrib["xyz"] == "0.25 0 0.65"
    camera = robot.find(
        "./gazebo[@reference='product_camera_link']/sensor[@name='product_camera']")
    assert camera.attrib["type"] == "rgbd_camera"
    assert float(camera.find("update_rate").text) == 10.0
    assert camera.find("camera/image/width").text == "640"
    assert camera.find("camera/image/height").text == "480"
    assert float(camera.find("camera/horizontal_fov").text) == pytest.approx(
        math.pi / 3.0)
    assert float(camera.find("camera/clip/near").text) == 0.1
    assert float(camera.find("camera/clip/far").text) == 5.0


def test_phase14_loaded_product_respects_mass_geometry_and_wrist_budget():
    robot = expand_composite(loaded=True)
    product = robot.find("./link[@name='stowed_product_link']")
    assert product.find("./collision/geometry/box").attrib["size"] == "0.30 0.20 0.15"
    product_mass = float(product.find("./inertial/mass").attrib["value"])
    assert product_mass == 5.0
    assert 0.8 + product_mass <= 6.0


def test_phase14_factory_composite_has_bilateral_contacts_and_detachable_products():
    robot = expand_factory_composite()
    for side in ("left", "right"):
        link = f"gripper_{side}_finger_link"
        sensor = robot.find(
            f"./gazebo[@reference='{link}']/sensor[@name='{side}_finger_contact']")
        assert sensor.attrib["type"] == "contact"
        assert float(sensor.find("update_rate").text) == 100.0
        assert sensor.find("contact/topic").text == (
            f"/amr/simulation/contacts/{side}_finger")
    plugins = robot.findall(
        "./gazebo/plugin[@name='gz::sim::systems::DetachableJoint']")
    assert [plugin.find("child_model").text for plugin in plugins] == [
        "product_a", "product_b", "product_c"]
    assert all(plugin.find("parent_link").text == "gripper_left_finger_link"
               for plugin in plugins)


def test_phase14_composite_control_and_moveit_contracts():
    robot = expand_composite()
    control = robot.find("./ros2_control[@name='Phase14MobileManipulatorSystem']")
    controlled = [joint.attrib["name"] for joint in control.findall("joint")]
    assert controlled == [f"arm_joint_{index}" for index in range(1, 7)] + [
        "gripper_finger_joint", "gripper_right_finger_joint"]
    right_joint = control.find("./joint[@name='gripper_right_finger_joint']")
    assert right_joint.find("./command_interface[@name='position']") is not None
    assert right_joint.find("./param[@name='mimic']") is None
    assert right_joint.find("./param[@name='multiplier']") is None

    controllers = yaml.safe_load(
        (ROOT / "config" / "phase14_mobile_manipulator_controllers.yaml").read_text())
    manager = controllers["controller_manager"]["ros__parameters"]
    assert manager["arm_controller"]["type"] == (
        "joint_trajectory_controller/JointTrajectoryController")
    assert manager["gripper_controller"]["type"] == (
        "position_controllers/GripperActionController")
    assert manager["gripper_right_controller"]["type"] == (
        "position_controllers/GripperActionController")
    left_gripper = controllers["gripper_controller"]["ros__parameters"]
    right_gripper = controllers["gripper_right_controller"]["ros__parameters"]
    assert right_gripper == {**left_gripper, "joint": "gripper_right_finger_joint"}
    arm_constraints = controllers["arm_controller"]["ros__parameters"]["constraints"]
    assert arm_constraints["goal_time"] == 1.0
    assert arm_constraints["stopped_velocity_tolerance"] == 0.01
    for joint in (f"arm_joint_{index}" for index in range(1, 7)):
        assert arm_constraints[joint] == {"trajectory": 0.05, "goal": 0.01}

    srdf = ET.parse(ROOT / "config" / "phase14_mobile_manipulator.srdf").getroot()
    assert "arm_pedestal_link" not in ET.tostring(srdf, encoding="unicode")
    chain = srdf.find("./group[@name='manipulator']/chain")
    assert chain.attrib == {"base_link": "arm_base_link", "tip_link": "gripper_tcp"}
    stowed = srdf.find("./group_state[@name='stowed']")
    values = {joint.attrib["name"]: float(joint.attrib["value"])
              for joint in stowed.findall("joint")}
    assert values == {f"arm_joint_{index + 1}": value for index, value in enumerate(
        (0.0, -1.5708, 1.5708, 0.0, 0.0, 0.0))}
    disabled = {(entry.attrib["link1"], entry.attrib["link2"])
                for entry in srdf.findall("disable_collisions")}
    assert ("base_link", "arm_base_link") in disabled
    assert not any(
        link1 == "base_link" and link2.startswith("arm_")
        and link2 != "arm_base_link"
        for link1, link2 in disabled
    )
    assert ("arm_link_6", "gripper_left_finger_link") in disabled
    assert ("arm_link_6", "gripper_right_finger_link") in disabled


def test_phase14_empty_and_loaded_stow_are_collision_free_and_inside_footprint():
    checker = (Path(get_package_prefix("amr_description")) / "lib" /
               "amr_description" / "phase14_collision_check")
    srdf = ROOT / "config" / "phase14_mobile_manipulator.srdf"
    for loaded in (False, True):
        urdf = subprocess.check_output(
            [
                "xacro",
                str(ROOT / "urdf" / "phase14_mobile_manipulator.urdf.xacro"),
                f"controller_config:={ROOT / 'config' / 'phase14_mobile_manipulator_controllers.yaml'}",
                f"loaded_product:={'true' if loaded else 'false'}",
            ]
        )
        with tempfile.NamedTemporaryFile(suffix=".urdf") as model:
            model.write(urdf)
            model.flush()
            subprocess.run([str(checker), model.name, str(srdf)], check=True)
