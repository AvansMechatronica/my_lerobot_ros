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

from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import ServoCommandType
from rclpy import qos
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
from std_srvs.srv import SetBool
from .config_ur import UrConfig

logger = logging.getLogger(__name__)


class Movegroup2ServoPose:
    """
    Python interface for MoveIt2 FollowJointTrajectory.
    """

    def __init__(
        self,
        node: "Node",
        config: UrConfig,
        callback_group: "CallbackGroup",
    ):
        self.config = config
        self._node = node
        self._frame_id = self.config.frame_id
        self._enabled = False

    def connect(self) -> None:
        pass

    def enable(self, wait_for_server_timeout_sec=1.0) -> bool:
        return True

    def disable(self, wait_for_server_timeout_sec=1.0) -> bool:
        pass

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        return {}

    def destroy(self) -> None:
        pass