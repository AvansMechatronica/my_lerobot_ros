from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig

from lerobot.robots.config import RobotConfig

@dataclass
class ROS2InterfaceConfig:
    namespace: str = ""
    arm_joint_names: list[str] = field(
        default_factory=lambda: [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
    )
    gripper_joint_name: str = "gripper_joint"
    base_link: str = "base_link"
    max_linear_velocity: float = 0.10
    max_angular_velocity: float = 0.25
    min_joint_positions: list[float] | None = field(default=None)
    max_joint_positions: list[float] | None = field(default=None)
    gripper_open_position: float = 0.0
    gripper_close_position: float = 1.0
    # Joint degree offsets (in degrees) telop --> arm
    target_degree_offsets: dict[str, float] = field(default_factory=lambda: {
        "shoulder_pan_joint": 0.0,
        "shoulder_lift_joint": -90.0,
        "elbow_joint": -90.0,
        "wrist_1_joint": 0.0,
        "wrist_2_joint": 90.0,
        "wrist_3_joint": 0.0,
    })
    # Joint scale factors (use -1.0 to invert direction) telop --> arm
    joint_scale_factors: dict[str, float] = field(default_factory=lambda: {
        "shoulder_pan_joint": 1.0,
        "shoulder_lift_joint": 1.0,
        "elbow_joint": 1.0,
        "wrist_1_joint": 1.0,
        "wrist_2_joint": 1.0,
        "wrist_3_joint": 1.0,
    })
    # Mapping from teleoperation joint names to actual joint names
    lookup_joint_names_from_telop: dict[str, str] = field(default_factory=lambda: {
        "shoulder_pan_joint": "shoulder_pan_joint",
        "shoulder_lift_joint": "shoulder_lift_joint",
        "elbow_joint": "elbow_joint",
        "wrist_1_joint": "wrist_1_joint",
        "wrist_2_joint": "wrist_2_joint",
        "wrist_3_joint": "wrist_3_joint",
    })
    trajectory_publisher: str = "/arm_controller/joint_trajectory"

@dataclass
class ROS2Config(RobotConfig):
    max_relative_target: int | None = None
    cameras: dict[str, CameraConfig] = field(default_factory=dict)
    ros2_interface: ROS2InterfaceConfig = field(default_factory=ROS2InterfaceConfig)

@RobotConfig.register_subclass("lerobot_robot_ur")
@dataclass
class UrConfig(ROS2Config):
    """Configuration for the Universal Robots UR5 with ROS 2."""
    ros2_interface: ROS2InterfaceConfig = field(
        default_factory=lambda: ROS2InterfaceConfig(
            arm_joint_names=[
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ],
            gripper_joint_name="gripper_joint",
            base_link="base_link",
            min_joint_positions=[-6.283, -2.356, -3.141, -6.283, -6.283, -6.283],
            max_joint_positions=[6.283, 2.356, 3.141, 6.283, 6.283, 6.283],
            gripper_open_position=0.0,
            gripper_close_position=1.0,
            target_degree_offsets={
                "shoulder_pan_joint": 0.0,
                "shoulder_lift_joint": -90.0,
                "elbow_joint": -90.0,
                "wrist_1_joint": 0.0,
                "wrist_2_joint": 90.0,
                "wrist_3_joint": 0.0,
            },
            joint_scale_factors={
                "shoulder_pan_joint": 1.0,
                "shoulder_lift_joint": 1.0,
                "elbow_joint": 1.0,
                "wrist_1_joint": 1.0,
                "wrist_2_joint": 1.0,
                "wrist_3_joint": 1.0,
            },
        )
    )


#@RobotConfig.register_subclass("lerobot_robot_ur")
@dataclass
class UrConfigx(RobotConfig):
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


    # Joint degree offsets (in degrees)
    target_degree_offsets: dict[str, float] = field(default_factory=lambda: {
        "shoulder_pan_joint": 0.0,
        "shoulder_lift_joint": -90.0,
        "elbow_joint": -90.0,
        "wrist_1_joint": 0.0,
        "wrist_2_joint": 90.0,
        "wrist_3_joint": 0.0,
    })

    # Joint scale factors (use -1.0 to invert direction)
    joint_scale_factors: dict[str, float] = field(default_factory=lambda: {
        "shoulder_pan_joint": 1.0,
        "shoulder_lift_joint": 1.0,
        "elbow_joint": 1.0,
        "wrist_1_joint": 1.0,
        "wrist_2_joint": 1.0,
        "wrist_3_joint": 1.0,
    })


    gripper_joint_name: str = "gripper_joint"  # Update if you have a gripper joint name
    cameras: dict[str, CameraConfig] = field(default_factory=dict)

