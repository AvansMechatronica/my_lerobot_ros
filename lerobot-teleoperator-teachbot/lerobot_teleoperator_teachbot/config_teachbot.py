from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("lerobot_teleoperator_teachbot")
@dataclass
class TeachbotConfig(TeleoperatorConfig):
    use_gripper: bool = True
