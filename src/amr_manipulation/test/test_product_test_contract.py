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
        if isinstance(node, ast.FunctionDef) and node.name in {"_finite_values", "_dock_bias"}
    ]

    class PreparationError(RuntimeError):
        pass

    namespace = {
        "math": math,
        "Iterable": Iterable,
        "Tuple": Tuple,
        "PreparationError": PreparationError,
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
    precise_egress = recovery_source.index(
        "self._navigate(self.selected.egress, precise=True)")
    egress_wait = recovery_source.index("self._wait_stationary()", precise_egress)
    precise_dock = recovery_source.index("self._navigate(self.selected.dock, precise=True)")
    assert relocalization_wait < missing_egress < precise_egress < egress_wait < precise_dock
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
