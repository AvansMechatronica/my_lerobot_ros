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
from control_msgs.action import FollowJointTrajectory

from .config_ur import UrConfig, ActionType


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
        self._joint_trajectory_client: ActionClient | None = None
        self.gripper_action_client: ActionClient | None = None
        self.gripper_traj_pub: Publisher | None = None
        self.executor: Executor | None = None
        self.executor_thread: threading.Thread | None = None
        self.is_connected = False
        #self._last_joint_state: dict[str, dict[str, float]] | None = None
        self._last_joint_state: dict[str, dict[str, float]] | None = {"position": {}, "velocity": {}}
        self.is_executing = False
        self._send_goal_future: Any = None
        self._current_goal_handle = None

    def connect(self) -> None:
        if not rclpy.ok():
            rclpy.init()

        self.robot_node = Node("ur_interface_node")

        # Spin the node briefly to allow topic discovery
        temp_executor = SingleThreadedExecutor()
        temp_executor.add_node(self.robot_node)

        if self.config.ros2_interface.action_type == ActionType.JOINT_POSITION:
            self.traj_cmd_pub = self.robot_node.create_publisher(
                JointTrajectory, self.config.ros2_interface.trajectory_publisher, 10
            )
        elif self.config.ros2_interface.action_type == ActionType.JOINT_TRAJECTORY:
            controller_name = self.config.ros2_interface.joint_trajectory_controller_sim if self.config.ros2_interface.sim else self.config.ros2_interface.joint_trajectory_controller
            self._joint_trajectory_client = ActionClient(
                self.robot_node,
                FollowJointTrajectory,
                controller_name,
            )
            if not self._joint_trajectory_client.wait_for_server(timeout_sec=5.0):
                raise ConnectionError("Joint trajectory action server is not available.") 
        else:
            raise ValueError(f"Unsupported action type: {self.config.ros2_interface.action_type}")       

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

    def goal_response_callback(self, future):
        """Handle goal response."""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                print('Goal rejected by action server')
                print('Possible reasons: controller not ready, invalid trajectory, or joints mismatch')
                print('Check that the controller is running: ros2 control list_controllers')
                self.is_executing = False
                return
            
            #print('Goal accepted')
        except Exception as e:
            print(f'Exception in goal response: {str(e)}')
            self.is_executing = False
            return
        
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handle result."""
        from control_msgs.action import FollowJointTrajectory as FJT
        result = future.result().result
        
        # Map error codes to messages
        error_messages = {
            FJT.Result.SUCCESSFUL: 'SUCCESSFUL',
            FJT.Result.INVALID_GOAL: 'INVALID_GOAL',
            FJT.Result.INVALID_JOINTS: 'INVALID_JOINTS',
            FJT.Result.OLD_HEADER_TIMESTAMP: 'OLD_HEADER_TIMESTAMP',
            FJT.Result.PATH_TOLERANCE_VIOLATED: 'PATH_TOLERANCE_VIOLATED',
            FJT.Result.GOAL_TOLERANCE_VIOLATED: 'GOAL_TOLERANCE_VIOLATED'
        }
        
        error_msg = error_messages.get(result.error_code, f'UNKNOWN({result.error_code})')
        
        if result.error_code == FJT.Result.SUCCESSFUL:
            #print(f'Result: {error_msg}')
            pass
        else:
            print(f'Result: {error_msg} (code: {result.error_code})')
        
        self.is_executing = False

    def feedback_callback(self, feedback_msg):
        """Handle feedback (optional)."""
        pass

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

        if self.config.ros2_interface.action_type == ActionType.JOINT_POSITION:
            # sent by publishig to topic
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
            #print(f"Publishing joint positions: {joint_positions}")
            self.traj_cmd_pub.publish(msg)
        elif self.config.ros2_interface.action_type == ActionType.JOINT_TRAJECTORY:
            # Create and send a FollowJointTrajectory action goal
            if self._joint_trajectory_client is None:
                raise DeviceNotConnectedError("Joint trajectory action client is not initialized.")
            # Check if the previous action is busy, then abort the previous goal
            if self._current_goal_handle is not None:
                self._current_goal_handle.cancel_goal_async()
                self._current_goal_handle = None
            goal_msg = FollowJointTrajectory.Goal()
            arm_joint_names = self.config.ros2_interface.arm_joint_names
            if len(joint_positions) != len(arm_joint_names):
                raise ValueError(
                    f"Expected {len(arm_joint_names)} joint positions, but got {len(joint_positions)}."
                )
            goal_msg.trajectory.joint_names = arm_joint_names
            point = JointTrajectoryPoint()
            point.positions = joint_positions
            point.time_from_start.sec = 1  # Set a default duration for the trajectory
            goal_msg.trajectory.points = [point]
            #print(f"Sending FollowJointTrajectory action goal: {goal_msg}")
            
            self.is_executing = True

            # Send the goal and store the future
            self._send_goal_future = self._joint_trajectory_client.send_goal_async(
                goal_msg,
                feedback_callback=self.feedback_callback
            )
            # When the goal response is received, store the goal handle
            def _store_goal_handle(future):
                goal_handle = future.result()
                self._current_goal_handle = goal_handle
                self.goal_response_callback(future)

            self._send_goal_future.add_done_callback(_store_goal_handle)
        else:
            raise ValueError(f"Unsupported action type: {self.config.ros2_interface.action_type}")


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
