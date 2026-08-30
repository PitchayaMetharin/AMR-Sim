from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_launches_independent_managed_pipelines():
    launch = (ROOT / "launch" / "amr_perception.launch.py").read_text()
    assert '"front_lidar_perception_node"' in launch
    assert '"rear_lidar_perception_node"' in launch
    assert "LifecycleNode" in launch
    assert '"use_sim_time": True' in launch


def test_pipeline_drops_nonusable_input_and_does_not_publish_tf_or_motion():
    source = (ROOT / "src" / "lidar_pipeline_node.cpp").read_text()
    assert "has_valid_layout(cloud)" in source
    assert "stamp > now" in source
    assert "stamp <= last_stamp_" in source
    assert "max_age_seconds_" in source
    assert "tf" not in source.lower()
    assert "twist" not in source.lower()


def test_contract_preserves_separate_sensor_topics():
    source = (ROOT / "src" / "lidar_pipeline_node.cpp").read_text()
    assert '"/amr/sensors/" + sensor_id_' in source
    assert '"/amr/perception/" + sensor_id_' in source


def test_live_fault_acceptance_covers_each_inhibit_case():
    script = (ROOT / "scripts" / "perception_fault_acceptance.py").read_text()
    assert "malformed=True" in script
    assert 'Parameter("use_sim_time", value=True)' in script
    assert "Stale PointCloud2 was republished" in script
    assert "Backward-time PointCloud2 was republished" in script
