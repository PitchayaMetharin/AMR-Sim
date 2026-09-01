"""Launch the factory graph with static-map AMCL or a mapping-mode shell."""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription
from launch.actions import OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable
from launch.actions import TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.events import matches_action
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from lifecycle_msgs.msg import Transition
import xacro


_DEFERRED_FACTORY_ACTIONS = []


def managed_node(package, executable):
    node = LifecycleNode(
        package=package,
        executable=executable,
        name=executable,
        namespace="/amr",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )
    configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(node),
        transition_id=Transition.TRANSITION_CONFIGURE,
    ))
    activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=node,
        goal_state="inactive",
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(node),
            transition_id=Transition.TRANSITION_ACTIVATE,
        ))],
    ))
    return [node, activate, configure]


def accessible_render_devices(dri_path="/dev/dri"):
    """Return DRM render nodes that the launch process can read and write."""
    if not os.path.isdir(dri_path):
        return []
    try:
        names = os.listdir(dri_path)
    except OSError:
        return []
    return [
        os.path.join(dri_path, name)
        for name in sorted(names)
        if name.startswith("renderD")
        and os.access(os.path.join(dri_path, name), os.R_OK | os.W_OK)
    ]


def software_renderer_forced(environment=None):
    """Report whether the environment explicitly selects software OpenGL."""
    environment = os.environ if environment is None else environment
    always_software = environment.get("LIBGL_ALWAYS_SOFTWARE", "").strip().lower()
    gallium_driver = environment.get("GALLIUM_DRIVER", "").strip().lower()
    return always_software in {"1", "true", "yes", "on"} or gallium_driver == "llvmpipe"


def validate_hardware_rendering(require_hardware, software_rendering,
                                environment=None, dri_path="/dev/dri"):
    """Fail before launch when an evidence run cannot use hardware rendering."""
    if not require_hardware:
        return []
    if software_rendering == "true":
        raise RuntimeError(
            "require_hardware_rendering=true conflicts with software_rendering=true")
    if software_renderer_forced(environment):
        raise RuntimeError(
            "hardware rendering required, but the environment forces software OpenGL")
    devices = accessible_render_devices(dri_path)
    if not devices:
        raise RuntimeError(
            "hardware rendering required, but no readable/writable /dev/dri/renderD* "
            "device is available")
    return devices


def validate_launch_options(context):
    """Reject invalid mode values before starting any factory process."""
    control_mode = LaunchConfiguration("control_mode").perform(context).lower()
    if control_mode not in {"manual", "autonomous"}:
        raise RuntimeError("control_mode must be manual or autonomous")
    mapping_mode = LaunchConfiguration("mapping_mode").perform(context).lower()
    if mapping_mode not in {"true", "false"}:
        raise RuntimeError("mapping_mode must be true or false")
    if not LaunchConfiguration("map_yaml").perform(context).strip():
        raise RuntimeError("map_yaml must be a non-empty path")


def launch_gazebo(context):
    factory = get_package_share_directory("amr_factory")
    server_only = LaunchConfiguration("headless").perform(context).lower() == "true"
    factory_attachment = LaunchConfiguration("factory_attachment").perform(context).lower()
    if factory_attachment not in {"true", "false"}:
        raise RuntimeError("factory_attachment must be true or false")
    software_rendering = LaunchConfiguration("software_rendering").perform(context).lower()
    require_hardware = (
        LaunchConfiguration("require_hardware_rendering").perform(context).lower() == "true"
    )
    if software_rendering not in {"auto", "true", "false"}:
        raise RuntimeError(
            "software_rendering must be one of: auto, true, false")

    render_devices = validate_hardware_rendering(
        require_hardware, software_rendering)
    render_device_available = bool(render_devices)
    use_software_rendering = (
        software_rendering == "true"
        or (software_rendering == "auto" and not render_device_available)
    )
    if require_hardware:
        print(
            "Hardware rendering required; accessible DRM render devices: "
            + ", ".join(render_devices))

    log_root = os.environ.get("ROS_LOG_DIR")
    if log_root:
        runtime_root = os.path.join(os.path.abspath(log_root), "gazebo_runtime")
    else:
        runtime_root = os.path.join(
            tempfile.gettempdir(), f"amr_gazebo_{os.getuid()}")
    cache_dir = os.path.join(runtime_root, "cache")
    config_dir = os.path.join(runtime_root, "config")
    runtime_dir = os.path.join(runtime_root, "runtime")
    gazebo_log_dir = os.path.join(runtime_root, "logs")
    for path in (cache_dir, config_dir, runtime_dir, gazebo_log_dir):
        os.makedirs(path, exist_ok=True)
    os.chmod(runtime_dir, 0o700)

    world_path = os.path.join(factory, "worlds", "factory.sdf")
    server_arguments = ["-r", "-s", world_path]
    # The server starts its update loop so dynamically spawned systems can
    # initialize. In native attachment mode Gate 6 still owns motion: the
    # bootstrap pauses the world before issuing any detach or pose-reset
    # request, then releases it only after its bounded evidence checks pass.
    gui_arguments = [
        "-g",
        "--render-engine-gui", "ogre2",
        "--render-engine-gui-api-backend", "opengl",
    ]
    model_path = os.path.join(factory, "models")
    resources = [
        model_path,
        os.path.dirname(get_package_share_directory("kuka_agilus_support")),
        os.path.dirname(get_package_share_directory("amr_description")),
    ]
    existing_resources = os.environ.get("GZ_SIM_RESOURCE_PATH")
    if existing_resources:
        resources.append(existing_resources)
    sdf_paths = [model_path]
    existing_sdf_path = os.environ.get("SDF_PATH")
    if existing_sdf_path:
        sdf_paths.append(existing_sdf_path)
    actions = [
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH", value=os.pathsep.join(resources)),
        SetEnvironmentVariable(name="SDF_PATH", value=os.pathsep.join(sdf_paths)),
        SetEnvironmentVariable(name="XDG_CACHE_HOME", value=cache_dir),
        SetEnvironmentVariable(name="XDG_CONFIG_HOME", value=config_dir),
        SetEnvironmentVariable(name="XDG_RUNTIME_DIR", value=runtime_dir),
        SetEnvironmentVariable(name="GZ_LOG_PATH", value=gazebo_log_dir),
    ]
    if not server_only:
        actions.append(SetEnvironmentVariable(
            name="QT_X11_NO_MITSHM", value="1"))
    if use_software_rendering:
        actions.extend([
            SetEnvironmentVariable(name="LIBGL_ALWAYS_SOFTWARE", value="1"),
            SetEnvironmentVariable(name="GALLIUM_DRIVER", value="llvmpipe"),
        ])
    gz_launch = PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory("ros_gz_sim"),
        "launch",
        "gz_sim.launch.py",
    ))
    actions.append(
        IncludeLaunchDescription(
            gz_launch,
            launch_arguments={"gz_args": " ".join(server_arguments)}.items(),
        )
    )
    if not server_only:
        actions.append(TimerAction(
            period=2.0,
            actions=[IncludeLaunchDescription(
                gz_launch,
                launch_arguments={"gz_args": " ".join(gui_arguments)}.items(),
            )],
        ))
    return actions


def launch_robot(context):
    """Create the robot graph after resolving launch-time xacro options."""
    deferred_factory_actions = list(_DEFERRED_FACTORY_ACTIONS)
    description = get_package_share_directory("amr_description")
    factory_attachment = LaunchConfiguration("factory_attachment").perform(context).lower()
    if factory_attachment not in {"true", "false"}:
        raise RuntimeError("factory_attachment must be true or false")

    robot_xml = xacro.process_file(
        os.path.join(description, "urdf", "phase14_mobile_manipulator.urdf.xacro"),
        mappings={
            "controller_config": os.path.join(
                description, "config", "phase14_mobile_manipulator_controllers.yaml"),
            "loaded_product": "false",
            "factory_attachment": factory_attachment,
            "joint_state_topic": "/world/factory_world/model/amr/joint_state",
        },
    ).toxml()
    robot_description = {"robot_description": robot_xml, "use_sim_time": True}

    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        remappings=[("joint_states", "/amr/base/joint_states")],
        output="screen",
    )
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "amr", "-x", LaunchConfiguration("initial_x"),
            "-y", LaunchConfiguration("initial_y"),
            "-Y", LaunchConfiguration("initial_yaw"),
            "-param", "robot_description",
        ],
        parameters=[robot_description],
        output="screen",
    )
    joint_states = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager-timeout", "30"],
        output="screen",
    )
    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager-timeout", "30"],
        output="screen",
    )
    gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager-timeout", "30"],
        output="screen",
    )
    gripper_right_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_right_controller", "--controller-manager-timeout", "30"],
        output="screen",
    )
    bootstrap = Node(
        package="amr_factory",
        executable="gate6_attachment_bootstrap",
        name="gate6_attachment_bootstrap",
        namespace="/amr",
        parameters=[{
            "use_sim_time": True,
            "initial_x": ParameterValue(LaunchConfiguration("initial_x"), value_type=float),
            "initial_y": ParameterValue(LaunchConfiguration("initial_y"), value_type=float),
            "initial_yaw": ParameterValue(LaunchConfiguration("initial_yaw"), value_type=float),
        }],
        output="screen",
    )
    pose_proxy = Node(
        package="amr_factory",
        executable="gazebo_set_pose_proxy",
        name="gazebo_set_pose_proxy",
        parameters=[{
            "gazebo_service": "/world/factory_world/set_pose",
            "request_timeout_ms": 3000,
        }],
        condition=IfCondition(LaunchConfiguration("factory_attachment")),
        output="screen",
    )
    control_proxy = Node(
        package="amr_factory",
        executable="gazebo_control_world_proxy",
        name="gazebo_control_world_proxy",
        parameters=[{
            "gazebo_service": "/world/factory_world/control",
            "request_timeout_ms": 3000,
        }],
        condition=IfCondition(LaunchConfiguration("factory_attachment")),
        output="screen",
    )
    ready_gate = Node(
        package="amr_factory",
        executable="gate6_bootstrap_ready_gate",
        name="gate6_bootstrap_ready_gate",
        parameters=[{"use_sim_time": False}],
        condition=IfCondition(LaunchConfiguration("factory_attachment")),
        output="screen",
    )
    pause_gate = Node(
        package="amr_factory",
        executable="gate6_bootstrap_pause_gate",
        name="gate6_bootstrap_pause_gate",
        parameters=[{"use_sim_time": False}],
        condition=IfCondition(LaunchConfiguration("factory_attachment")),
        output="screen",
    )
    inserted_gate = Node(
        package="amr_factory",
        executable="gate6_bootstrap_inserted_gate",
        name="gate6_bootstrap_inserted_gate",
        parameters=[{"use_sim_time": False}],
        condition=IfCondition(LaunchConfiguration("factory_attachment")),
        output="screen",
    )
    controller_ready_gate = Node(
        package="amr_factory",
        executable="gate6_controller_ready_gate",
        name="gate6_controller_ready_gate",
        parameters=[{"use_sim_time": False}],
        output="screen",
    )
    actions = [
        state_publisher,
    ]
    if factory_attachment == "true":
        # Start the bootstrap before insertion.  Its PAUSED status is the
        # insertion gate, and the create-exit handshake lets it queue detach
        # commands before the first physics step, preventing the stock
        # DetachableJoint systems from racing controller startup; detach commands are queued before the first physics step.
        actions.append(pose_proxy)
        actions.append(control_proxy)
        actions.append(ready_gate)
        actions.append(bootstrap)
        actions.append(pause_gate)
        actions.append(RegisterEventHandler(
            OnProcessExit(target_action=pause_gate, on_exit=[spawn])))
        actions.append(RegisterEventHandler(
            OnProcessExit(target_action=spawn, on_exit=[inserted_gate])))
        actions.append(RegisterEventHandler(
            OnProcessExit(target_action=ready_gate, on_exit=[joint_states])))
        actions.append(RegisterEventHandler(
            OnProcessExit(target_action=joint_states, on_exit=[arm_controller])))
        actions.append(RegisterEventHandler(
            OnProcessExit(target_action=arm_controller, on_exit=[gripper_controller])))
        # The native attachment bootstrap must prove READY before the
        # controller chain starts.  Keep the sensor/navigation graph deferred
        # until all four controllers are active so its startup load cannot
        # starve the Gazebo controller-manager callback thread.
        actions.append(RegisterEventHandler(
            OnProcessExit(target_action=gripper_controller, on_exit=[gripper_right_controller])))
        actions.append(RegisterEventHandler(
            OnProcessExit(target_action=gripper_right_controller, on_exit=[controller_ready_gate])))
        actions.append(RegisterEventHandler(
            OnProcessExit(target_action=controller_ready_gate,
                          on_exit=deferred_factory_actions)))
    else:
        actions.extend([
            # Give robot_state_publisher time to expose robot_description before
            # gz_ros2_control requests it while the spawned entity is configured.
            TimerAction(period=2.0, actions=[spawn]),
            RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[joint_states])),
            RegisterEventHandler(
                OnProcessExit(target_action=joint_states, on_exit=[arm_controller])),
            RegisterEventHandler(
                OnProcessExit(target_action=arm_controller, on_exit=[gripper_controller])),
            RegisterEventHandler(
                OnProcessExit(target_action=gripper_controller, on_exit=[gripper_right_controller])),
            RegisterEventHandler(
                OnProcessExit(target_action=gripper_right_controller, on_exit=[controller_ready_gate])),
            RegisterEventHandler(
                OnProcessExit(target_action=controller_ready_gate,
                              on_exit=deferred_factory_actions)),
        ])
    return actions


def generate_launch_description():
    global _DEFERRED_FACTORY_ACTIONS
    factory = get_package_share_directory("amr_factory")
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/model/amr/cmd_vel@geometry_msgs/msg/TwistStamped]gz.msgs.Twist",
            "/model/amr/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/model/amr/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose",
            "/world/factory_world/model/amr/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model",
            "/amr/simulation/sensors/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/amr/simulation/sensors/front_lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/amr/simulation/sensors/rear_lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/amr/simulation/sensors/front_lidar/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "/amr/simulation/sensors/rear_lidar/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "/amr/simulation/sensors/product_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/amr/simulation/sensors/product_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/amr/simulation/sensors/product_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/amr/simulation/contacts/left_finger@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
            "/amr/simulation/contacts/right_finger@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
            "/model/product_a/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose",
            "/model/product_b/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose",
            "/model/product_c/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose",
            "/amr/simulation/internal/attachment/product_101/attach@std_msgs/msg/Empty]gz.msgs.Empty",
            "/amr/simulation/internal/attachment/product_101/detach@std_msgs/msg/Empty]gz.msgs.Empty",
            "/amr/simulation/internal/attachment/product_101/state@std_msgs/msg/String[gz.msgs.StringMsg",
            "/amr/simulation/internal/attachment/product_102/attach@std_msgs/msg/Empty]gz.msgs.Empty",
            "/amr/simulation/internal/attachment/product_102/detach@std_msgs/msg/Empty]gz.msgs.Empty",
            "/amr/simulation/internal/attachment/product_102/state@std_msgs/msg/String[gz.msgs.StringMsg",
            "/amr/simulation/internal/attachment/product_103/attach@std_msgs/msg/Empty]gz.msgs.Empty",
            "/amr/simulation/internal/attachment/product_103/detach@std_msgs/msg/Empty]gz.msgs.Empty",
            "/amr/simulation/internal/attachment/product_103/state@std_msgs/msg/String[gz.msgs.StringMsg",
        ],
        remappings=[
            ("/model/amr/cmd_vel", "/amr/simulation/base/cmd_vel"),
            ("/model/amr/odometry", "/amr/simulation/base/odometry"),
            ("/model/amr/pose", "/amr/simulation/ground_truth/pose"),
            ("/world/factory_world/model/amr/joint_state", "/amr/simulation/base/joint_states"),
            ("/amr/simulation/sensors/front_lidar/scan/points", "/amr/simulation/sensors/front_lidar/points"),
            ("/amr/simulation/sensors/rear_lidar/scan/points", "/amr/simulation/sensors/rear_lidar/points"),
        ],
    )

    amcl_parameters = os.path.join(factory, "config", "amcl.yaml")
    map_path = os.path.join(factory, "maps", "factory.yaml")
    map_server = LifecycleNode(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        namespace="/amr",
        parameters=[amcl_parameters, {
            "yaml_filename": LaunchConfiguration("map_yaml"),
        }],
        remappings=[("map", "/map"), ("map_metadata", "/map_metadata")],
        condition=UnlessCondition(LaunchConfiguration("mapping_mode")),
        output="screen",
    )
    amcl = LifecycleNode(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        namespace="/amr",
        # The spawn pose and AMCL pose are the same launch-authoritative
        # values.  ParameterValue preserves their numeric type when launch
        # substitutions are resolved.
        parameters=[amcl_parameters, {
            "initial_pose": {
                "x": ParameterValue(LaunchConfiguration("initial_x"), value_type=float),
                "y": ParameterValue(LaunchConfiguration("initial_y"), value_type=float),
                "z": 0.0,
                "yaw": ParameterValue(LaunchConfiguration("initial_yaw"), value_type=float),
            },
            "set_initial_pose": True,
        }],
        remappings=[("map", "/map"), ("scan", "/amr/sensors/front_lidar/scan")],
        condition=UnlessCondition(LaunchConfiguration("mapping_mode")),
        output="screen",
    )
    localization_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_factory_localization",
        namespace="/amr",
        parameters=[amcl_parameters],
        condition=UnlessCondition(LaunchConfiguration("mapping_mode")),
        output="screen",
    )

    autonomous_condition = IfCondition(PythonExpression([
        "'", LaunchConfiguration("control_mode"), "' == 'autonomous'",
    ]))

    include_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("amr_localization"),
            "launch", "amr_localization.launch.py")))
    include_perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("amr_perception"),
            "launch", "amr_perception.launch.py")))
    include_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("amr_navigation"),
            "launch", "amr_navigation.launch.py")),
        condition=autonomous_condition)
    include_mpc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("amr_mpc_controller"),
            "launch", "amr_mpc_controller.launch.py")),
        condition=autonomous_condition)
    include_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("amr_control"),
            "launch", "amr_control.launch.py")),
        launch_arguments={"require_manipulator_stowed": "true"}.items())
    include_mission = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("amr_mission"),
            "launch", "amr_mission.launch.py")),
        condition=autonomous_condition)
    product_tag_detector = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="product_tag_detector",
        namespace="/amr",
        parameters=[os.path.join(factory, "config", "apriltag.yaml")],
        remappings=[
            ("image_rect", "/amr/sensors/product_camera/image_rect"),
            ("camera_info", "/amr/sensors/product_camera/camera_info"),
            ("detections", "/amr/perception/product_tags"),
        ],
        output="screen",
    )

    deferred_factory_actions = [
        map_server,
        amcl,
        include_localization,
        include_perception,
        product_tag_detector,
    ]

    actions = [
        # Fast DDS synchronous service replies can block in the publisher and
        # intermittently time out during the multi-node lifecycle bringup.
        # Keep the transport reliable, but queue replies asynchronously for
        # this high-load factory graph.  This is scoped to child processes of
        # this launch and does not alter any topic QoS or readiness requirement.
        SetEnvironmentVariable(
            name="RMW_FASTRTPS_PUBLICATION_MODE", value="ASYNCHRONOUS"),
        DeclareLaunchArgument("headless", default_value="true"),
        # The factory graph includes GPU LiDAR and an RGB-D camera.  Do not
        # silently run that timing-sensitive workload on llvmpipe: fail fast
        # when the host cannot provide a hardware renderer.  Software remains
        # available as an explicit, non-acceptance-test override.
        DeclareLaunchArgument("software_rendering", default_value="false"),
        DeclareLaunchArgument("require_hardware_rendering", default_value="true"),
        # Products are placed on shelves and must remain detached in the
        # ordinary factory simulation.  Gate 6 opts in explicitly because it
        # exercises the native attach/detach interface.
        DeclareLaunchArgument("factory_attachment", default_value="false"),
        DeclareLaunchArgument("control_mode", default_value="autonomous"),
        DeclareLaunchArgument("mapping_mode", default_value="false"),
        DeclareLaunchArgument("map_yaml", default_value=map_path),
        OpaqueFunction(function=validate_launch_options),
        DeclareLaunchArgument("initial_x", default_value="-4.5"),
        DeclareLaunchArgument("initial_y", default_value="0.0"),
        DeclareLaunchArgument("initial_yaw", default_value="0.0"),
        OpaqueFunction(function=launch_gazebo),
        bridge,
        OpaqueFunction(function=launch_robot),
    ]
    base_node, base_activate, base_configure = managed_node(
        "amr_base_adapter", "base_adapter_node")
    front_node, front_activate, front_configure = managed_node(
        "amr_sensor_adapters", "front_lidar_adapter_node")
    rear_node, rear_activate, rear_configure = managed_node(
        "amr_sensor_adapters", "rear_lidar_adapter_node")
    imu_node, imu_activate, imu_configure = managed_node(
        "amr_sensor_adapters", "imu_adapter_node")
    product_camera_node, product_camera_activate, product_camera_configure = managed_node(
        "amr_sensor_adapters", "product_camera_adapter_node")

    # Register every adapter and its inactive->active handler before starting
    # the ordered configure chain.  A failed transition stops the chain.
    deferred_factory_actions.extend([
        base_node,
        base_activate,
        front_node,
        front_activate,
        rear_node,
        rear_activate,
        imu_node,
        imu_activate,
        product_camera_node,
        product_camera_activate,
    ])
    deferred_factory_actions.append(TimerAction(period=8.0, actions=[base_configure]))
    deferred_factory_actions.extend([
        RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=base_node,
            goal_state="active",
            entities=[front_configure],
        )),
        RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=front_node,
            goal_state="active",
            entities=[rear_configure],
        )),
        RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=rear_node,
            goal_state="active",
            entities=[imu_configure],
        )),
        RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=imu_node,
            goal_state="active",
            entities=[product_camera_configure],
        )),
        # Start the localization manager and command arbitration only after
        # every adapter is active.  The final adapter state is the causal
        # readiness boundary for the wheel/joint and sensor topics needed by
        # the localization and control consumers.
        RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=product_camera_node,
            goal_state="active",
            entities=[
                # Keep command arbitration and the localization lifecycle
                # manager out of the same discovery/response burst.  These
                # are bounded startup delays only; both still start causally
                # after every adapter is active.  The control timer also
                # preserves command arbitration in mapping mode, where AMCL
                # is intentionally absent.
                TimerAction(period=2.0, actions=[include_control]),
                TimerAction(period=4.0, actions=[localization_manager]),
            ],
        )),
        # AMCL owns map->odom in production mode.  Do not release Nav2 until
        # AMCL has reached active, so its costmaps cannot begin with a missing
        # map transform while their manager is already trying to activate.
        RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=amcl,
            goal_state="active",
            entities=[
                include_navigation,
                # Planning needs to settle before the controller lifecycle
                # manager joins the DDS transition traffic.  Mission starts
                # last so no command source is active during either burst.
                TimerAction(period=5.0, actions=[include_mpc]),
                TimerAction(period=8.0, actions=[include_mission]),
            ],
        )),
    ])
    _DEFERRED_FACTORY_ACTIONS = deferred_factory_actions
    return LaunchDescription(actions)
