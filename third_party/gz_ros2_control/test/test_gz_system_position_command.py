# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "src" / "gz_system.cpp").read_text()


def test_position_branch_uses_direct_position_reset():
    write_start = SOURCE.index(
        "hardware_interface::return_type GazeboSimSystem::write(")
    position_start = SOURCE.index(
        "} else if (this->dataPtr->joints_[i].joint_control_method & POSITION) {",
        write_start)
    effort_start = SOURCE.index(
        "} else if (this->dataPtr->joints_[i].joint_control_method & EFFORT) {",
        position_start)
    position_branch = SOURCE[position_start:effort_start]

    assert "JointPositionReset" in position_branch
    assert "joint_position_cmd" in position_branch
    assert "JointVelocityCmd" not in position_branch
    assert "position_proportional_gain_" not in position_branch


def test_velocity_and_effort_branches_keep_native_commands():
    write_start = SOURCE.index(
        "hardware_interface::return_type GazeboSimSystem::write(")
    velocity_start = SOURCE.index(
        "if (this->dataPtr->joints_[i].joint_control_method & VELOCITY) {",
        write_start)
    position_start = SOURCE.index(
        "} else if (this->dataPtr->joints_[i].joint_control_method & POSITION) {",
        velocity_start)
    effort_start = SOURCE.index(
        "} else if (this->dataPtr->joints_[i].joint_control_method & EFFORT) {",
        position_start)
    fallback_start = SOURCE.index(
        "} else if (this->dataPtr->joints_[i].is_actuated) {", effort_start)

    velocity_branch = SOURCE[velocity_start:position_start]
    effort_branch = SOURCE[effort_start:fallback_start]
    assert "JointVelocityCmd" in velocity_branch
    assert "joint_velocity_cmd" in velocity_branch
    assert "JointForceCmd" in effort_branch
    assert "joint_effort_cmd" in effort_branch
