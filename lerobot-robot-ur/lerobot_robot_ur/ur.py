import logging
from typing import Any





from lerobot.cameras import make_cameras_from_configs
from lerobot.utils.errors import DeviceNotConnectedError, DeviceAlreadyConnectedError
from lerobot.robots.robot import Robot

from .config_ur import UrConfig


logger = logging.getLogger(__name__)


class Lite6Gripper:
    def __init__(self):
        pass
    def open(self):
        pass

    def close(self):
        pass

    def stop(self):
        pass
    def set_gripper_state(self, gripper_state: float) -> None:
        pass
    def get_gripper_state(self) -> float:
        return 0

    def reset_gripper(self) -> None:
        pass


class Ur(Robot):
    config_class = UrConfig
    name = "ur"

    def __init__(self, config: UrConfig):
        super().__init__(config)
        self.cameras = make_cameras_from_configs(config.cameras)
        self._gripper = Lite6Gripper()
        self._is_connected = False
        


    def connect(self, calibrate: bool = True) -> None:
        pass

    @property
    def _motors_ft(self) -> dict[str, type]:
        # Example: Provide at least one action feature (customize as needed)
        return {"joint_1": float}

    @property
    def action_features(self) -> dict[str, type]:
        return self._motors_ft

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        print("Robot action:", action)
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        return action

    def get_observation(self) -> dict[str, Any]:

        obs_dict = {}
        return obs_dict

    def reset(self):
        if self._gripper:
            self._gripper.reset_gripper()

    def disconnect(self) -> None:
        if not self.is_connected:
            return

        if self._gripper is not None:
            self._gripper.stop()
            self._gripper = None

        for cam in self.cameras.values():
            cam.disconnect()

        self.is_connected = False
        logger.info(f"{self} disconnected.")

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass
    def is_calibrated(self) -> bool:
        return True#self.is_connected

    @property
    def is_connected(self) -> bool:
        return True#self._is_connected

    @is_connected.setter
    def is_connected(self, value: bool) -> None:
        self._is_connected = value

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3)
            for cam in self.cameras
        }

    @property
    def observation_features(self) -> dict[str, Any]:
        features = {**self._motors_ft, **self._cameras_ft}
        return features

    @property
    def cameras(self):
        return self._cameras

    @cameras.setter
    def cameras(self, value):
        self._cameras = value

    @property
    def config(self):
        return self._config

    @config.setter
    def config(self, value):
        self._config = value
