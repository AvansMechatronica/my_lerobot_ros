from typing import Any
from lerobot.teleoperators.teleoperator import Teleoperator
from .config_teachbot import TeachbotConfig
from teleop import Teleop as SpesTeleop
import numpy as np
import threading
from .ros_interface import ROS2Interface


class Teachbot(Teleoperator):
    config_class = TeachbotConfig
    name = "teachbot"

    def __init__(self, config: TeachbotConfig):
        super().__init__(config)
        self.config = config
        self.ros2_interface = ROS2Interface(config=config)
        self._connected = False
        self._calibrated = True
        self._home = False


        self._mutex = threading.Lock()

    def _on_teleop_callback(self, pose, message):
        pass

    @property
    def action_features(self) -> dict[str, type]:
        return{}


    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._connected

    def connect(self, calibrate: bool = True) -> None:
        self.ros2_interface.connect()
        self._connected = True

    @property
    def is_calibrated(self) -> bool:
        return self._calibrated

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def get_action(self) -> dict[str, Any]:
        action = self.ros2_interface.joint_state
        #print("Teachbot action:", action)
        return action

    def send_feedback(self, feedback: dict[str, float]) -> None:
        pass

    def disconnect(self) -> None: 
        self.ros2_interface.disconnect()
        self._connected = False
        self._calibrated = False
        self._home = False
