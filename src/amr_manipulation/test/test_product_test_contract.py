import ast
import math
from pathlib import Path
from types import SimpleNamespace
from typing import Iterable, Tuple

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
RUNNER = PACKAGE_ROOT / "scripts" / "gate6_product_test.py"


def _runner_module():
    tree = ast.parse(RUNNER.read_text(encoding="utf-8"))
    functions = [
        node for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name in {
            "_finite_values", "_xy_distance", "_dock_travel_target",
            "_recoverable_dock_abort", "_dock_bias"
        }
    ]

    class PreparationError(RuntimeError):
        pass

    namespace = {
        "math": math,
        "Iterable": Iterable,
        "Tuple": Tuple,
        "PreparationError": PreparationError,
        "DOCK_POSITION_TOLERANCE_M": 0.03,
        "DOCK_CORRECTION_MAX_POSITION_M": 0.15,
        "DOCK_CORRECTION_MAX_YAW_RAD": 0.15,
    }
    exec(compile(ast.Module(body=functions, type_ignores=[]), str(RUNNER), "exec"), namespace)
    return SimpleNamespace(**namespace)


def test_persistent_product_runner_is_limited_and_fail_closed():
    source = RUNNER.read_text(encoding="utf-8")

    assert "RESET_PRODUCT_IDS = (102, 103)" in source
    assert '"/world/factory_world/control"' in source
    assert '"/world/factory_world/set_pose"' in source
    assert "self._set_world_paused(True)" in source
    assert "self._set_selected_product_pose()" in source
    assert "self._verify_attachment_bootstrap()" in source
    bootstrap_start = source.index("def _verify_attachment_bootstrap")
    bootstrap_end = source.index("def _wait_selected_pose_stable", bootstrap_start)
    bootstrap_source = source[bootstrap_start:bootstrap_end]
    assert 'rclpy.create_node(\n            "gate6_bootstrap_verify_client", context=self.context)' in bootstrap_source
    assert "bootstrap_node.create_client(" in bootstrap_source
    assert "executor.add_node(bootstrap_node)" in bootstrap_source
    assert "executor.spin_until_future_complete(future, timeout_sec=5.0)" in bootstrap_source
    assert "bootstrap_client.remove_pending_request(future)" in bootstrap_source
    assert "executor.remove_node(bootstrap_node)" in bootstrap_source
    assert "executor.shutdown(timeout_sec=0.0)" in bootstrap_source
    assert "bootstrap_node.destroy_node()" in bootstrap_source
    assert "self._bootstrap_client" not in source
    assert source.index("self._verify_attachment_bootstrap()") < source.index("self._reset_selected_product()")
    assert '"fresh attachment states"' not in source
    assert "diagnostic_qos" in source
    assert "depth=20" in source
    assert "deadline=Duration(seconds=0.1)" in source
    assert "goal_handle.get_result_async()" in source
    assert "client.get_result_async(goal_handle)" not in source
    assert "depth=1" in source
    assert "self._detach_all()" not in source
    assert '"/amr/mission/navigate_to_pose"' in source
    assert '"/amr/mission/navigate_to_pose_precise"' in source
    assert '"/amr/mission/navigate_to_pose_retreat"' in source
    assert "self._retreat_navigation" in source
    assert "self._set_status(ManipulatorStatus.FAULT, False, False, detail)" in source
    assert '"gate6_mass_stage.launch.py"' in source
    assert "product_id:=unknown" not in source
    assert "feedback_callback=feedback_callback" in source
    assert "egress: Optional[Tuple[float, float, float]]" in source
    assert 'station.get("egress")' in source
    assert "def _retreat_from_other_pickup_dock" in source
    assert "        self._retreat_from_other_pickup_dock()" in source
    assert "metadata.egress" in source
    assert "AMR is at pickup dock for product" in source
    assert "navigation endpoint {endpoint} to" in source
    assert "class NavigationAbortedError(PreparationError)" in source
    assert "result.status != GoalStatus.STATUS_ABORTED" in source
    assert "raise NavigationAbortedError(detail, latest_localized)" in source
    assert source.count("except NavigationAbortedError as error:") == 2
    assert "if dock_navigation_aborted or" in source
    assert "_dock_bias(" in source
    assert "physical_dock, localized_dock)" in source
    assert '"/amr/set_initial_pose"' in source
    assert '"/amr/amcl_pose"' in source
    assert 'PoseWithCovarianceStamped, "/amr/amcl_pose", self._amcl_pose_callback, amcl_qos' in source
    assert "durability=DurabilityPolicy.TRANSIENT_LOCAL" in source
    assert "request.pose.pose.covariance[0]" in source
    assert "new converged AMCL pose after {label.lower()} relocalization" in source
    assert "no AMCL sample was received during dock navigation" in source
    assert "RELOCALIZATION_TERMINAL_AMCL_MAX_AGE_S = 6.0" in source
    assert "AMCL terminal pose from recovery navigation is too old" in source
    assert "base is not stationary for {label.lower()} relocalization" in source
    assert "Dock localization discrepancy bounded before recovery" in source
    assert "Let the localization transform propagate" in source
    assert "Recover clearance using the registered, collision-checked" in source
    assert "self._navigate(self.selected.approach, precise=True, retreat=True)" in source
    assert "localized_approach = self._navigate(self.selected.approach, precise=False)" in source
    assert "physical_approach, localized_approach" in source
    assert "self._relocalize_at_reference(" in source
    assert "no AMCL sample was received during approach recovery" in source
    assert "self._navigate(self.selected.dock, precise=True)" in source
    recovery_start = source.index("self._relocalize_at_reference(")
    recovery_end = source.index("self._verify_dock_and_product_geometry()", recovery_start)
    recovery_source = source[recovery_start:recovery_end]
    relocalization_wait = recovery_source.index("self._wait_stationary()")
    missing_egress = recovery_source.index(
        'if self.selected.egress is None:\n'
        '                    raise PreparationError("selected pickup station has no registered egress")')
    normal_egress = recovery_source.index(
        "self._navigate(self.selected.egress, precise=False)")
    egress_wait = recovery_source.index("self._wait_stationary()", normal_egress)
    precise_dock = recovery_source.index("self._navigate(self.selected.dock, precise=True)")
    assert relocalization_wait < missing_egress < normal_egress < egress_wait < precise_dock
    assert "self._navigate(self.selected.dock, precise=False)" not in recovery_source
    assert "self._amcl_pose_generation += 1" in source
    assert "baseline_generation = self._amcl_pose_generation" in source
    assert "self._amcl_pose_generation <= baseline_generation" in source
    assert "SingleThreadedExecutor(context=self.context)" in source
    assert "executor.spin_once(timeout_sec=0.05)" in source
    assert "executor.remove_node(self)" in source
    assert "executor.shutdown(timeout_sec=0.0)" in source
    assert "timeout_sec=min(" not in source
    assert "fresh physical dock pose" in source
    assert "DOCK_CORRECTION_MAX_POSITION_M" in source
    assert "DOCK_CORRECTION_MAX_YAW_RAD" in source


def test_recovery_precise_dock_abort_requires_fresh_independent_final_proof():
    source = RUNNER.read_text(encoding="utf-8")
    recovery_start = source.index("self._relocalize_at_reference(")
    proof = source.index("self._verify_dock_and_product_geometry()", recovery_start)
    recovery_source = source[recovery_start:proof]

    final_dock = recovery_source.index(
        "self._navigate(self.selected.dock, precise=True)")
    final_dock_try = recovery_source.rfind("try:", 0, final_dock)
    final_dock_catch = recovery_source.index(
        "except NavigationAbortedError as error:", final_dock)
    recovery_gate = recovery_source.index(
        "_recoverable_dock_abort(\n"
        "                            error.localized_pose, self.selected.dock)",
        final_dock_catch)
    recovery_log = recovery_source.index(
        "Recovery precise dock action aborted inside the existing ",
        final_dock_catch)
    stationary = recovery_source.index("self._wait_stationary()", recovery_log)
    fresh_pose = recovery_source.index(
        '"fresh physical dock pose after recovery abort"', stationary)

    assert final_dock_try < final_dock < final_dock_catch < recovery_gate
    assert recovery_gate < recovery_log < stationary < fresh_pose
    assert proof > fresh_pose
    assert "self._navigate(self.selected.dock, precise=True)" not in \
        recovery_source[final_dock_catch:]


def test_initial_precise_dock_waits_and_uses_registered_egress_clearance():
    source = RUNNER.read_text(encoding="utf-8")
    prepare_start = source.index("    def prepare(self) -> None:")
    first_dock_attempt = source.index(
        "            amcl_generation_before_dock =", prepare_start)
    route = source[prepare_start:first_dock_attempt]

    normal_approach = route.index(
        "self._navigate(self.selected.approach, precise=False)")
    egress_guard = route.index(
        'if self.selected.egress is None:\n'
        '                raise PreparationError("selected pickup station has no registered egress")',
        normal_approach)
    approach_settle = route.index(
        "self._wait_stationary()", egress_guard)
    egress_navigation = route.index(
        "self._navigate(self.selected.egress, precise=False)", egress_guard)
    egress_settle = route.index("self._wait_stationary()", egress_navigation)
    precise_dock = source.index(
        "localized_dock = self._navigate(self.selected.dock, precise=True)",
        first_dock_attempt)

    assert normal_approach < egress_guard < approach_settle
    assert egress_guard < egress_navigation < egress_settle < precise_dock
    assert "self._navigate(self.selected.dock, precise=False)" not in route


def test_final_dock_uses_travel_bearing_then_registered_terminal_heading():
    runner = _runner_module()
    travel_target = runner._dock_travel_target(
        (1.8303, 0.0172, -0.0854), (2.4, 0.0, 0.0))
    assert travel_target[:2] == pytest.approx((2.4, 0.0))
    assert travel_target[2] == pytest.approx(-0.030182, abs=1e-6)
    assert runner._dock_travel_target(
        (2.4, 0.0, 0.8), (2.4, 0.0, 0.0)) == (2.4, 0.0, 0.0)

    source = RUNNER.read_text(encoding="utf-8")
    assert "DOCK_POSITION_TOLERANCE_M = 0.03" in source
    assert "DOCK_YAW_TOLERANCE_RAD = 0.15" in source
    assert "PRODUCT_RELATIVE_TOLERANCE_M = 0.04" in source
    assert "def _verify_dock_and_product_geometry(self) -> None:" in source
    assert source.count("dock_travel_target = self._final_dock_travel_target()") == 2
    assert source.count("self._navigate(dock_travel_target, precise=True)") == 2
    assert source.count("self._navigate(self.selected.dock, precise=True)") == 2

    first_target = source.index(
        "dock_travel_target = self._final_dock_travel_target()")
    second_target = source.index(
        "dock_travel_target = self._final_dock_travel_target()", first_target + 1)
    first_travel = source.index(
        "self._navigate(dock_travel_target, precise=True)", first_target)
    first_heading = source.index(
        "self._navigate(self.selected.dock, precise=True)", first_travel)
    second_travel = source.index(
        "self._navigate(dock_travel_target, precise=True)", second_target)
    second_heading = source.index(
        "self._navigate(self.selected.dock, precise=True)", second_travel)
    assert first_travel < first_heading < second_target
    assert second_travel < second_heading


def test_dock_correction_uses_physical_minus_localized_bias():
    runner = _runner_module()
    bias = runner._dock_bias(
        (2.3205338, 0.0237318, 0.00308),
        (2.3961699, -0.0031411, 0.005261),
    )
    assert bias == pytest.approx((-0.0756361, 0.0268729, -0.002181), abs=1e-6)


def test_dock_correction_wraps_yaw_and_rejects_non_finite_or_unbounded_bias():
    runner = _runner_module()
    wrapped = runner._dock_bias(
        (0.0, 0.0, math.pi - 0.01),
        (0.0, 0.0, -math.pi + 0.02),
    )
    assert wrapped[2] == pytest.approx(-0.03, abs=1e-6)
    with pytest.raises(runner.PreparationError):
        runner._dock_bias((float("nan"), 0.0, 0.0), (0.0, 0.0, 0.0))
    with pytest.raises(runner.PreparationError):
        runner._dock_bias((0.2, 0.0, 0.0), (0.0, 0.0, 0.0))


def test_recoverable_dock_abort_uses_existing_position_window_only_as_recovery_gate():
    runner = _runner_module()

    # Retained Product 102 evidence: terminal XY reached the registered dock
    # window, while yaw still required the fail-closed recovery sequence.
    assert runner._recoverable_dock_abort(
        (2.3994, -0.0047, -0.2643), (2.4, 0.0, 0.0))
    assert not runner._recoverable_dock_abort(
        (2.4301, 0.0, 0.0), (2.4, 0.0, 0.0))
    with pytest.raises(runner.PreparationError):
        runner._recoverable_dock_abort(
            (float("nan"), 0.0, 0.0), (2.4, 0.0, 0.0))


def test_product_aliases_select_only_the_new_independent_tests():
    launch_3kg = (PACKAGE_ROOT / "launch" / "gate6_3kg_test.launch.py").read_text(
        encoding="utf-8")
    launch_5kg = (PACKAGE_ROOT / "launch" / "gate6_5kg_test.launch.py").read_text(
        encoding="utf-8")

    assert 'executable="gate6_product_test"' in launch_3kg
    assert '"product_id": 102' in launch_3kg
    assert 'executable="gate6_product_test"' in launch_5kg
    assert '"product_id": 103' in launch_5kg


def test_mass_stage_success_message_remains_compatible_for_one_kg():
    source = (PACKAGE_ROOT / "src" / "gate6_mass_stage.cpp").read_text(
        encoding="utf-8")

    assert '"GATE 6 %.1f KG COMPLETE %.0f KG PASS"' in source
    assert "product.mass_kg, product.mass_kg" in source


def test_factory_launch_uses_native_service_proxies_for_reset_control():
    source = (PACKAGE_ROOT.parent / "amr_factory" / "launch" /
              "factory_localization.launch.py").read_text(encoding="utf-8")

    assert 'executable="gazebo_control_world_proxy"' in source
    assert 'actions.append(control_proxy)' in source
    assert "/world/factory_world/control@ros_gz_interfaces/srv/ControlWorld" not in source
    assert 'executable="gazebo_set_pose_proxy"' in source
    assert "/world/factory_world/set_pose@ros_gz_interfaces/srv/SetEntityPose" not in source


def test_runner_is_installed_under_the_ros_executable_name():
    source = (PACKAGE_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")

    assert "scripts/gate6_product_test.py" in source
    assert "RENAME gate6_product_test" in source
