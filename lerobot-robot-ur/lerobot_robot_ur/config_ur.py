from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig

from lerobot.robots.config import RobotConfig

@RobotConfig.register_subclass("lerobot_robot_ur")
@dataclass
class UrConfig(RobotConfig):
    use_gripper: bool = True
    cameras: dict[str, CameraConfig] = field(default_factory=dict)
