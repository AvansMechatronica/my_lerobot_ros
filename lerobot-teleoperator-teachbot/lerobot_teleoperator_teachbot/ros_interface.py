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
from teachbot_interfaces.msg import TeachbotState

#from .config import ActionType, GripperActionType, ROS2InterfaceConfig
#from .moveit_servo import MoveIt2Servo
from .config_teachbot import TeachbotConfig

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

    def __init__(self, config: TeachbotConfig):
        self.config = config
        self.robot_node: Node | None = None
        self.executor: Executor | None = None
        self.executor_thread: threading.Thread | None = None
        self.is_connected = False
        self._last_joint_state: dict[str, dict[str, float]] | None = None
        self.teachbot_pot_precent = 0.0
        self.teachbot_btn1 = False
        self._teachbot_btn1_prev = False
        self.teachbot_btn2 = False

        self.teachbot_enabled = False

    def connect(self) -> None:
        if not rclpy.ok():
            rclpy.init()

        self.robot_node = Node("teachbot_teleop_interface_node")

        # Spin the node briefly to allow topic discovery
        temp_executor = SingleThreadedExecutor()
        temp_executor.add_node(self.robot_node)
        start_time = time.time()
        while time.time() - start_time < 1.0:
            temp_executor.spin_once(timeout_sec=0.1)
        temp_executor.remove_node(self.robot_node)

        if 0: # Dit werkt niet betrowbaar, omdat de topics soms vertraagd verschijnen. We checken in de joint state callback zelf of de benodigde joints er zijn.
            topics = self.robot_node.get_topic_names_and_types()
            #print("Available topics:", topics)
            has_joint_states = any(topic == "/teachbot/joint_states" for topic, _ in topics)
            has_state = any(topic == "/teachbot/state" for topic, _ in topics)
            if not (has_joint_states and has_state):
                logger.error("Required topics /teachbot/joint_states or /teachbot/state do not exist.")
                self.is_connected = False
                return

        self.joint_state_sub = self.robot_node.create_subscription(
            JointState,
            "/teachbot/joint_states",
            self._joint_state_callback,
            10,
        )

        self.state_sub = self.robot_node.create_subscription(
            TeachbotState,
            '/teachbot/state',
            self._teachbot_state_callback,
            10
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

    @property
    def joint_state(self) -> dict[str, dict[str, float]] | None:
        """Get the last received joint state."""
        #print("Getting joint state:", self._last_joint_state)
        return self._last_joint_state

    def _joint_state_callback(self, msg: "JointState") -> None:
        if self.teachbot_enabled:
            self._last_joint_state = self._last_joint_state or {}
            positions = {}
            name_to_index = {name: i for i, name in enumerate(msg.name)}
            for joint_name in self.config.arm_joint_names:
                # Try direct match
                idx = name_to_index.get(joint_name)
                # If not found, try with 'teachbot/' prefix
                if idx is None:
                    prefixed_name = f"teachbot/{joint_name}"
                    idx = name_to_index.get(prefixed_name)
                if idx is None:
                    raise ValueError(f"Joint '{joint_name}' (or 'teachbot/{joint_name}') not found in joint state.")
                positions[joint_name] = msg.position[idx]

            self._last_joint_state["position"] = positions
            if self.config.use_gripper:
                if self.config.vacuum_gripper:
                    if self.teachbot_pot_precent > 50.0:
                        self._last_joint_state["gripper"] = 100.0
                    else:
                        self._last_joint_state["gripper"] = 0.0
                else:
                    self._last_joint_state["gripper"] = self.teachbot_pot_precent
        else:
            self._last_joint_state = {}


    def _teachbot_state_callback(self, msg: TeachbotState):
        """Callback for teachbot state messages."""

        self.teachbot_pot_precent = msg.pistol.pot_percent
        self.teachbot_btn1 = msg.pistol.btn1
        self.teachbot_btn2 = msg.pistol.btn2

        # Toggle teachbot_enabled on btn1 rising edge
        if self.teachbot_btn1 and not self._teachbot_btn1_prev:
            self.teachbot_enabled = not self.teachbot_enabled
        self._teachbot_btn1_prev = self.teachbot_btn1


    def disconnect(self):
        if hasattr(self, "joint_state_sub") and self.joint_state_sub:
            self.joint_state_sub.destroy()
            self.joint_state_sub = None
        if hasattr(self, "state_sub") and self.state_sub:
            self.state_sub.destroy()
            self.state_sub = None
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
