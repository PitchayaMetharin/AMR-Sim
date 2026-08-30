from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "src" / "gazebo_set_pose_proxy.cpp"
LAUNCH = ROOT / "launch" / "factory_localization.launch.py"


def test_pose_proxy_uses_native_gazebo_pose_service_contract():
    source = SOURCE.read_text(encoding="utf-8")
    assert "gz/msgs/pose.pb.h" in source
    assert "gz/msgs/boolean.pb.h" in source
    assert "gazebo_node_.Request" in source
    assert "response->success = state->result && state->success" in source


def test_pose_proxy_is_only_started_for_native_attachment_mode():
    launch = LAUNCH.read_text(encoding="utf-8")
    assert 'condition=IfCondition(LaunchConfiguration("factory_attachment"))' in launch
    assert '"gazebo_service": "/world/factory_world/set_pose"' in launch
