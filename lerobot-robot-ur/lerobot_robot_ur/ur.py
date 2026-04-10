import logging
from typing import Any
import time


from lerobot.cameras import make_cameras_from_configs
from lerobot.utils.errors import DeviceNotConnectedError, DeviceAlreadyConnectedError
from lerobot.robots.robot import Robot
from lerobot.robots.utils import ensure_safe_goal_position

from .config_ur import UrConfig
from .ros_interface_ur import ROS2Interface


logger = logging.getLogger(__name__)


class Ur(Robot):
    config_class = UrConfig
    name = "ur"

    def __init__(self, config: UrConfig):
        super().__init__(config)
        self.config = config
        self.ros2_interface = ROS2Interface(config=config)
        self.cameras = make_cameras_from_configs(config.cameras)
        self.is_connected = False

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} is already connected.")
        self.ros2_interface.connect()
        self.is_connected = True
        #logger.info(f"{self} connected.")


    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        """Command arm to move to a target joint configuration.

        The relative action magnitude may be clipped depending on the configuration parameter
        `max_relative_target`. In this case, the action sent differs from original action.
        Thus, this function always returns the action actually sent.

        Args:
            action (dict[str, float]): The goal positions for the motors or pressed_keys dict.

        Raises:
            DeviceNotConnectedError: if robot is not connected.

        Returns:
            dict[str, float]: The action sent to the motors, potentially clipped.
        """
        #print(f"Received action: {action}" )
        if 1:
            if not self.is_connected:
                raise DeviceNotConnectedError(f"{self} is not connected.")

            if self.config.max_relative_target is not None:
                goal_present_pos = {}
                joint_state = self.ros2_interface.joint_state
                if joint_state is None:
                    raise ValueError("Joint state is not available yet.")

                for key, goal in action.items():
                    present_pos = joint_state["position"].get(key.replace(".pos", ""), 0.0)
                    goal_present_pos[key] = (goal, present_pos)
                action = ensure_safe_goal_position(goal_present_pos, self.config.max_relative_target)


            # Use lookup_joint_names_from_telop for mapping
            lookup = getattr(self.config.ros2_interface, "lookup_joint_names_from_telop", {})
            offsets = getattr(self.config.ros2_interface, "target_degree_offsets", {})
            scales = getattr(self.config.ros2_interface, "joint_scale_factors", {})
            arm_joint_names = self.config.ros2_interface.arm_joint_names

            joint_positions = []
            for arm_joint in arm_joint_names:
                # Find the teleop joint name that maps to this arm joint
                teleop_joint = None
                for k, v in lookup.items():
                    if v == arm_joint:
                        teleop_joint = k
                        break
                # Try both .pos and plain key
                teleop_val = action.get(f"{teleop_joint}.pos", action.get(teleop_joint, 0.0)) if teleop_joint else 0.0
                offset_deg = offsets.get(arm_joint, 0.0)
                offset = offset_deg * 3.141592653589793 / 180.0  # convert degrees to radians
                scale = scales.get(arm_joint, 1.0)
                arm_val = (teleop_val + offset) * scale
                joint_positions.append(arm_val)
            #print(f"Mapped joint positions: {joint_positions}")
            self.ros2_interface.send_joint_position_command(joint_positions)

            #gripper_pos = action["gripper.pos"]
            #self.ros2_interface.send_gripper_command(gripper_pos)
        return action


    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        obs_dict: dict[str, Any] = {}
        joint_state = self.ros2_interface.joint_state
        if joint_state is None or "position" not in joint_state:
            logger.warning("Joint state 'position' not available yet.")
        else:
            obs_dict.update({f"{joint}.pos": pos for joint, pos in joint_state["position"].items()})

        # Capture images from cameras
        for cam_key, cam in self.cameras.items():
            start = time.perf_counter()
            try:
                obs_dict[cam_key] = cam.async_read(timeout_ms=300)
            except Exception as e:
                logger.error(f"Failed to read camera {cam_key}: {e}")
                obs_dict[cam_key] = None
            dt_ms = (time.perf_counter() - start) * 1e3
            logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")

        return obs_dict

    def reset(self):
        pass

    def disconnect(self) -> None:
        if not self.is_connected:
            return

        self.ros2_interface.disconnect()

        from lerobot.utils.errors import DeviceNotConnectedError
        for cam in self.cameras.values():
            try:
                cam.disconnect()
            except DeviceNotConnectedError:
                logger.warning(f"Tried to disconnect {cam}, but it was not connected.")

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
        return self._is_connected

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
        features = {**self._cameras_ft}
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

    @property
    def action_features(self) -> dict[str, type]:
        # Provide action features for each arm joint
        features = {f"{joint}.pos": float for joint in self.config.ros2_interface.arm_joint_names}
        # Optionally add gripper
        if hasattr(self.config.ros2_interface, "gripper_joint_name"):
            features[f"{self.config.ros2_interface.gripper_joint_name}.pos"] = float
        return features
