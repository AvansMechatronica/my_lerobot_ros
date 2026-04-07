from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("teachbot")
@dataclass
class TeachbotTeleopConfig(TeleoperatorConfig):
    use_gripper: bool = True
