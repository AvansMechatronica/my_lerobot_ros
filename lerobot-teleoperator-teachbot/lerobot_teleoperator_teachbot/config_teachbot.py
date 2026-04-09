from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig


from dataclasses import dataclass, field

@TeleoperatorConfig.register_subclass("lerobot_teleoperator_teachbot")
@dataclass
class TeachbotConfig(TeleoperatorConfig):
    use_gripper: bool = True
    vacuum_gripper: bool = False  # Set to True if using a vacuum gripper
    arm_joint_names: list[str] = field(default_factory=lambda: [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint"
    ])
    gripper_joint_name: str = "gripper_joint"  # Update if you have a gripper joint name
