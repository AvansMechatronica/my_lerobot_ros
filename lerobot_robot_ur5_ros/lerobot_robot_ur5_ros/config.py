# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
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

from dataclasses import dataclass, field
from enum import Enum

from lerobot.cameras import CameraConfig
from lerobot.robots import RobotConfig

class ActionType(Enum):
    CARTESIAN_VELOCITY = "cartesian_velocity"
    JOINT_POSITION = "joint_position"
    JOINT_TRAJECTORY = "joint_trajectory"

class GripperActionType(Enum):
    TRAJECTORY = "trajectory"
    ACTION = "action"

@dataclass
class ROS2InterfaceConfig:
    namespace: str = ""
    arm_joint_names: list[str] = field(
        default_factory=lambda: [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
    )
    gripper_joint_name: str = "gripper_joint"
    base_link: str = "base_link"
    max_linear_velocity: float = 0.10
    max_angular_velocity: float = 0.25
    min_joint_positions: list[float] | None = field(default=None)
    max_joint_positions: list[float] | None = field(default=None)
    gripper_open_position: float = 0.0
    gripper_close_position: float = 1.0
    gripper_action_type: GripperActionType = GripperActionType.TRAJECTORY

@dataclass
class ROS2Config(RobotConfig):
    action_type: ActionType = ActionType.JOINT_POSITION
    max_relative_target: int | None = None
    cameras: dict[str, CameraConfig] = field(default_factory=dict)
    ros2_interface: ROS2InterfaceConfig = field(default_factory=ROS2InterfaceConfig)

@RobotConfig.register_subclass("ur5_ros")
@dataclass
class UR5ROSConfig(ROS2Config):
    """Configuration for the Universal Robots UR5 with ROS 2."""
    action_type: ActionType = ActionType.JOINT_POSITION
    ros2_interface: ROS2InterfaceConfig = field(
        default_factory=lambda: ROS2InterfaceConfig(
            arm_joint_names=[
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ],
            gripper_joint_name="gripper_joint",
            base_link="base_link",
            min_joint_positions=[-6.283, -2.356, -3.141, -6.283, -6.283, -6.283],
            max_joint_positions=[6.283, 2.356, 3.141, 6.283, 6.283, 6.283],
            gripper_open_position=0.0,
            gripper_close_position=1.0,
            gripper_action_type=GripperActionType.TRAJECTORY,
        ),
    )
