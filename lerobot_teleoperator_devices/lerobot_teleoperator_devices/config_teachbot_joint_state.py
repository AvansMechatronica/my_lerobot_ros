from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("teachbot")
@dataclass
class TeachbotTeleopConfig(TeleoperatorConfig):
    """
    Configuration for the TeachbotJointStateTeleop device.
    Set use_gripper to True if the teleoperator should control a gripper.
    """
    use_gripper: bool = True
