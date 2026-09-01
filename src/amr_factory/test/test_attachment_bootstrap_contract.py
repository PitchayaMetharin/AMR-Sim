from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH = PACKAGE_ROOT / "launch" / "factory_localization.launch.py"
SCRIPT = PACKAGE_ROOT / "scripts" / "gate6_attachment_bootstrap.py"
READY_GATE = PACKAGE_ROOT / "scripts" / "gate6_bootstrap_ready_gate.py"
PAUSE_GATE = PACKAGE_ROOT / "scripts" / "gate6_bootstrap_pause_gate.py"
INSERTED_GATE = PACKAGE_ROOT / "scripts" / "gate6_bootstrap_inserted_gate.py"
CONTROLLER_GATE = PACKAGE_ROOT / "scripts" / "gate6_controller_ready_gate.py"


def test_bootstrap_is_only_enabled_for_native_attachment_mode():
    launch = LAUNCH.read_text(encoding="utf-8")
    assert 'server_arguments = ["-r", "-s", world_path]' in launch
    assert 'if factory_attachment == "true":' in launch
    assert "server starts its update loop" in launch
    assert 'executable="gate6_attachment_bootstrap"' in launch
    assert 'executable="gazebo_set_pose_proxy"' in launch
    assert 'executable="gate6_bootstrap_ready_gate"' in launch
    assert 'executable="gate6_bootstrap_pause_gate"' in launch
    assert 'executable="gate6_bootstrap_inserted_gate"' in launch
    assert 'executable="gate6_controller_ready_gate"' in launch
    assert 'actions.append(pose_proxy)' in launch
    assert 'OnProcessExit(target_action=pause_gate, on_exit=[spawn])' in launch
    assert 'OnProcessExit(target_action=spawn, on_exit=[inserted_gate])' in launch
    assert 'OnProcessExit(target_action=ready_gate, on_exit=[joint_states])' in launch
    assert 'OnProcessExit(target_action=gripper_controller, on_exit=[gripper_right_controller])' in launch
    assert 'OnProcessExit(target_action=gripper_right_controller, on_exit=[controller_ready_gate])' in launch
    assert 'on_exit=deferred_factory_actions' in launch
    assert '"/world/factory_world/set_pose@ros_gz_interfaces/srv/SetEntityPose"' not in launch
    assert 'actions.append(bootstrap)' in launch
    assert 'detach commands are queued before the first physics step' in launch


def test_bootstrap_is_fail_closed_and_restores_registered_product_poses():
    source = SCRIPT.read_text(encoding="utf-8")
    assert "ControlWorld" in source
    assert '"/amr/simulation/base/joint_states"' in source
    assert "multi_step = 1 if step else 0" in source
    assert "STEP_CLOCK_TIMEOUT_S" in source
    assert "paused physics step did not advance the simulation clock" in source
    assert "newer clock value" in source
    assert "_queue_detach_and_step()" in source
    assert "event-driven transition topic" in source
    assert 'self._attachment_states[product_id] in {"attached", "detached"}' in source
    assert "bounded live sensor-validation window" in source
    assert "PRODUCT_SETTLE_SIM_SECONDS = 0.25" in source
    assert "settling product contacts" in source
    assert "self._sim_time - release_sim < PRODUCT_SETTLE_SIM_SECONDS" in source
    assert "_check_robot_and_stow()" in source
    assert "Product contact settle complete; starting strict" in source
    assert "self._check_products(drift_reference)" in source
    assert "PRODUCT_POSITION_TOLERANCE_M = 0.005" in source
    assert "PRODUCT_DRIFT_TOLERANCE_M = 0.005" in source
    assert "start_sim: Optional[float] = None" in source
    assert "self._set_world_paused(False)" in source
    assert "_confirm_world_release()" in source
    assert "Final startup release requested" in source
    assert "POST_RELEASE_CONFIRM_SIM_SECONDS" in source
    assert "MIN_POST_RELEASE_JOINT_STAMPS" in source
    assert "_release_joint_stamp_count >= MIN_POST_RELEASE_JOINT_STAMPS" in source
    assert "newer joint-state stamps" in source
    assert "_set_product_pose(product)" in source
    assert "self._set_world_paused(True)" in source
    assert 'self._state = "FAULT"' in source
    assert 'self._state = "READY"' in source
    assert '"/amr/simulation/attachment_bootstrap/verify"' in source
    assert '"/amr/simulation/attachment_bootstrap/status"' in source
    assert '"/amr/simulation/attachment_bootstrap/robot_inserted"' in source
    assert 'self._state = "PAUSED"' in source


def test_insertion_gates_hold_on_fault():
    pause_source = PAUSE_GATE.read_text(encoding="utf-8")
    inserted_source = INSERTED_GATE.read_text(encoding="utf-8")
    assert 'status.startswith("PAUSED ")' in pause_source
    assert 'status.startswith("FAULT ")' in pause_source
    assert "refusing robot insertion" in pause_source
    assert 'SERVICE = "/amr/simulation/attachment_bootstrap/robot_inserted"' in inserted_source
    assert "wait_for_service(timeout_sec=10.0)" in inserted_source


def test_controller_gate_holds_deferred_graph_on_failure():
    launch = LAUNCH.read_text(encoding="utf-8")
    source = CONTROLLER_GATE.read_text(encoding="utf-8")
    controller_gate_block = launch.split(
        "    controller_ready_gate = Node(", 1)[1].split(
            "    actions = [", 1)[0]
    attachment_modes = launch.split(
        '    if factory_attachment == "true":', 1)[1]
    false_mode_block = attachment_modes.split("    else:", 1)[1].split(
        "    return actions", 1)[0]
    assert "condition=IfCondition" not in controller_gate_block
    assert "deferred_factory_actions" in launch
    assert "OnProcessExit(target_action=gripper_controller, on_exit=[gripper_right_controller])" in false_mode_block
    assert "OnProcessExit(target_action=gripper_right_controller, on_exit=[controller_ready_gate])" in false_mode_block
    assert "OnProcessExit(target_action=controller_ready_gate" in false_mode_block
    assert "on_exit=deferred_factory_actions" in false_mode_block
    assert "actions.extend(deferred_factory_actions)" not in false_mode_block
    assert "REQUIRED_CONTROLLERS" in source
    assert '"gripper_right_controller"' in source
    assert "CONTROLLER_SERVICE_TIMEOUT_SEC = 5.0" in source
    assert "controller-manager service" in source
    assert "refusing deferred factory graph" in source
    assert "time.monotonic()" in source


def test_bootstrap_checks_launch_pose_and_empty_stow():
    source = SCRIPT.read_text(encoding="utf-8")
    assert 'self.declare_parameter("initial_x", 0.0)' in source
    assert 'self.declare_parameter("initial_y", 0.0)' in source
    assert 'self.declare_parameter("initial_yaw", 0.0)' in source
    assert "AMR moved during attachment bootstrap" in source
    assert "arm left empty stow during attachment bootstrap" in source


def test_controller_startup_is_gated_by_latched_bootstrap_ready():
    source = READY_GATE.read_text(encoding="utf-8")
    assert 'STATUS_TOPIC = "/amr/simulation/attachment_bootstrap/status"' in source
    assert "TRANSIENT_LOCAL" in source
    assert 'status.startswith("READY ")' in source
    assert 'status.startswith("FAULT ")' in source
    assert "this process alive on FAULT" in source
