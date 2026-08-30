"""Launch the primitive AMR Gazebo plant and ROS simulation boundaries."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from launch.substitutions import LaunchConfiguration
import xacro


def managed_node(package, executable, name):
    node = LifecycleNode(package=package, executable=executable, name=name, namespace="/amr", parameters=[{"use_sim_time": True}])
    configure = EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(node), transition_id=Transition.TRANSITION_CONFIGURE))
    activate = RegisterEventHandler(OnStateTransition(target_lifecycle_node=node, goal_state="inactive", entities=[EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(node), transition_id=Transition.TRANSITION_ACTIVATE))]))
    return node, configure, activate


def launch_gazebo(context):
    simulation = get_package_share_directory("amr_simulation")
    server_only = LaunchConfiguration("headless").perform(context).lower() == "true"
    arguments = "-r " + ("-s " if server_only else "") + os.path.join(
        simulation, "worlds", "amr_world.sdf")
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": arguments}.items())]


def generate_launch_description():
    description = get_package_share_directory("amr_description")
    localization = get_package_share_directory("amr_localization")
    perception = get_package_share_directory("amr_perception")
    slam = get_package_share_directory("amr_slam")
    navigation = get_package_share_directory("amr_navigation")
    mission = get_package_share_directory("amr_mission")
    mpc_controller = get_package_share_directory("amr_mpc_controller")
    control = get_package_share_directory("amr_control")
    robot_xml = xacro.process_file(os.path.join(description, "urdf", "amr.urdf.xacro")).toxml()
    gazebo = OpaqueFunction(function=launch_gazebo)
    state_publisher = Node(package="robot_state_publisher", executable="robot_state_publisher", parameters=[{"robot_description": robot_xml, "use_sim_time": True}])
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", "amr", "-param", "robot_description"],
        parameters=[{"robot_description": robot_xml}],
        output="screen",
    )
    bridge = Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen", arguments=[
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock", "/model/amr/cmd_vel@geometry_msgs/msg/TwistStamped]gz.msgs.Twist", "/model/amr/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry", "/model/amr/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose", "/world/amr_world/model/amr/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model", "/amr/simulation/sensors/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU", "/amr/simulation/sensors/front_lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan", "/amr/simulation/sensors/rear_lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan", "/amr/simulation/sensors/front_lidar/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked", "/amr/simulation/sensors/rear_lidar/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked"], remappings=[("/model/amr/cmd_vel", "/amr/simulation/base/cmd_vel"), ("/model/amr/odometry", "/amr/simulation/base/odometry"), ("/model/amr/pose", "/amr/simulation/ground_truth/pose"), ("/world/amr_world/model/amr/joint_state", "/amr/simulation/base/joint_states"), ("/amr/simulation/sensors/front_lidar/scan/points", "/amr/simulation/sensors/front_lidar/points"), ("/amr/simulation/sensors/rear_lidar/scan/points", "/amr/simulation/sensors/rear_lidar/points")])
    managed = [managed_node("amr_base_adapter", "base_adapter_node", "base_adapter_node"), managed_node("amr_sensor_adapters", "front_lidar_adapter_node", "front_lidar_adapter_node"), managed_node("amr_sensor_adapters", "rear_lidar_adapter_node", "rear_lidar_adapter_node"), managed_node("amr_sensor_adapters", "imu_adapter_node", "imu_adapter_node"), managed_node("amr_health", "health_supervisor_node", "health_supervisor_node")]
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            localization, "launch", "amr_localization.launch.py")))
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            perception, "launch", "amr_perception.launch.py")))
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            slam, "launch", "amr_slam.launch.py")))
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            navigation, "launch", "amr_navigation.launch.py")))
    mpc_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            mpc_controller, "launch", "amr_mpc_controller.launch.py")))
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            control, "launch", "amr_control.launch.py")))
    mission_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            mission, "launch", "amr_mission.launch.py")))
    actions = [DeclareLaunchArgument("headless", default_value="false"), gazebo, state_publisher, spawn, bridge, localization_launch, perception_launch, slam_launch, navigation_launch, mpc_controller_launch, control_launch, mission_launch]
    for node, configure, activate in managed:
        actions.extend([node, configure, activate])
    return LaunchDescription(actions)
