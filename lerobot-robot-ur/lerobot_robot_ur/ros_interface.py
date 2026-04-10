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

import logging
import threading
import time

import rclpy
from control_msgs.action import GripperCommand
from lerobot.utils.errors import DeviceNotConnectedError
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import Executor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.publisher import Publisher
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .config_ur import UrConfig


logger = logging.getLogger(__name__)


class ROS2Interface:
    """Class to interface with a MoveIt2 manipulator.

    This class supports both JointGroupPositionController and JointTrajectoryController
    from ros2_control for arm control, depending on the configuration:

    - ActionType.JOINT_POSITION:
      Uses JointGroupPositionController.
      Publishes Float64MultiArray messages to '/position_controller/commands'

    - ActionType.JOINT_TRAJECTORY:
      Uses JointTrajectoryController.
      Publishes JointTrajectory messages to '/arm_controller/joint_trajectory'

    The gripper control also supports both trajectory and action-based control
    via the gripper_action_type configuration option.
    """

    def __init__(self, config: UrConfig):
        self.config = config
        self.robot_node: Node | None = None
        self.pos_cmd_pub: Publisher | None = None
        self.traj_cmd_pub: Publisher | None = None
        self.gripper_action_client: ActionClient | None = None
        self.gripper_traj_pub: Publisher | None = None
        self.executor: Executor | None = None
        self.executor_thread: threading.Thread | None = None
        self.is_connected = False
        #self._last_joint_state: dict[str, dict[str, float]] | None = None
        self._last_joint_state: dict[str, dict[str, float]] | None = {"position": {}, "velocity": {}}

    def connect(self) -> None:
        if not rclpy.ok():
            rclpy.init()

        self.robot_node = Node("ur_interface_node")

        # Spin the node briefly to allow topic discovery
        temp_executor = SingleThreadedExecutor()
        temp_executor.add_node(self.robot_node)

        self.traj_cmd_pub = self.robot_node.create_publisher(
            JointTrajectory, self.config.ros2_interface.trajectory_publisher, 10
        )

        self.joint_state_sub = self.robot_node.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_callback,
            10,
        )

        # Create and start the executor in a separate thread
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.robot_node)
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()
        time.sleep(3)  # Give some time to connect to services and receive messages

        self.is_connected = True

    def send_joint_position_command(self, joint_positions: list[float], unnormalize: bool = True) -> None:
        """
        Send a command to the robot's joints.
        Args:
            joint_positions (list[float]): The target positions for the joints.
            unnormalize (bool): Whether to unnormalize the joint positions based on the robot's configuration.
        """
        if not self.robot_node:
            raise DeviceNotConnectedError("ROS2Interface is not connected. You need to call `connect()`.")

        if unnormalize:
            min_joint_positions = self.config.ros2_interface.min_joint_positions
            max_joint_positions = self.config.ros2_interface.max_joint_positions
            if min_joint_positions is None or max_joint_positions is None:
                raise ValueError(
                    "Joint position normalization requires min and max joint positions to be set."
                )
            joint_positions = [
                min(max(pos, min_pos), max_pos)
                for pos, min_pos, max_pos in zip(
                    joint_positions,
                    min_joint_positions,
                    max_joint_positions,
                    strict=True,
                )
            ]

        arm_joint_names = self.config.ros2_interface.arm_joint_names
        if len(joint_positions) != len(arm_joint_names):
            raise ValueError(
                f"Expected {len(arm_joint_names)} joint positions, but got {len(joint_positions)}."
            )

        if self.traj_cmd_pub is None:
            raise DeviceNotConnectedError("Trajectory command publisher is not initialized.")
        msg = JointTrajectory()
        arm_joint_names = self.config.ros2_interface.arm_joint_names
        msg.joint_names = arm_joint_names
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        msg.points = [point]
        print(f"Publishing joint positions: {joint_positions}")
        #self.traj_cmd_pub.publish(msg)


    @property
    def joint_state(self) -> dict[str, dict[str, float]] | None:
        """Get the last received joint state."""
        return self._last_joint_state

    def _joint_state_callback(self, msg: "JointState") -> None:
        #print(f"Received joint state: {msg}")
        self._last_joint_state = self._last_joint_state or {}
        positions = {}
        velocities = {}
        name_to_index = {name: i for i, name in enumerate(msg.name)}
        for joint_name in self.config.ros2_interface.arm_joint_names:
            idx = name_to_index.get(joint_name)
            if idx is None:
                raise ValueError(f"Joint '{joint_name}' not found in joint state.")
            positions[joint_name] = msg.position[idx]
            velocities[joint_name] = 0.0 #msg.velocity[idx]

        self._last_joint_state["position"] = positions
        self._last_joint_state["velocity"] = velocities
        #print(f"Updated joint state: {self._last_joint_state}")

    def disconnect(self):
        if self.joint_state_sub:
            self.joint_state_sub.destroy()
            self.joint_state_sub = None
        if self.pos_cmd_pub:
            self.pos_cmd_pub.destroy()
            self.pos_cmd_pub = None
        if self.traj_cmd_pub:
            self.traj_cmd_pub.destroy()
            self.traj_cmd_pub = None
        if self.gripper_traj_pub:
            self.gripper_traj_pub.destroy()
            self.gripper_traj_pub = None
        if self.robot_node:
            self.robot_node.destroy_node()
            self.robot_node = None

        if self.executor:
            self.executor.shutdown()
            self.executor = None
        if self.executor_thread:
            self.executor_thread.join()
            self.executor_thread = None

        self.is_connected = False
