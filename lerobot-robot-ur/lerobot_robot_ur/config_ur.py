from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig

from lerobot.robots.config import RobotConfig
from enum import Enum

class ActionType(Enum):
    CARTESIAN_VELOCITY = "cartesian_velocity"
    JOINT_POSITION = "joint_position"
    MOVEGROUP_FOLLOW_JOINT_TRAJECTION = "movegroup_follow_joint_trajectory"
    MOVEGROUP_SERVO_TWIST = "movegroup_servo_twist"
    MOVEGROUP_SERVO_POSE = "movegroup_servo_pose"
    MOVEGROUP_SERVO_JOG = "movegroup_servo_jog"

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
    # UR5/UR10 joint limits in radians (example: UR5)
    min_joint_positions: list[float] = field(default_factory=lambda: [
        -6.283185307179586,  # shoulder_pan_joint
        -2.356194490192345,  # shoulder_lift_joint
        -3.141592653589793,  # elbow_joint
        -6.283185307179586,  # wrist_1_joint
        -6.283185307179586,  # wrist_2_joint
        -6.283185307179586,  # wrist_3_joint
    ])
    max_joint_positions: list[float] = field(default_factory=lambda: [
        6.283185307179586,   # shoulder_pan_joint
        0.0,                # shoulder_lift_joint
        3.141592653589793,  # elbow_joint
        6.283185307179586,  # wrist_1_joint
        6.283185307179586,  # wrist_2_joint
        6.283185307179586,  # wrist_3_joint
    ])
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
    action_type: ActionType = ActionType.JOINT_POSITION
    trajectory_publisher: str = "/arm_controller/joint_trajectory"
    joint_trajectory_controller_sim: str = "/joint_trajectory_controller/follow_joint_trajectory"
    joint_trajectory_controller: str = "/scaled_joint_trajectory_controller/follow_joint_trajectory"
    joint_position_controller_commands: str = "/forward_position_controller/commands"
    servo_delta_joint_cmds: str = "/servo_node/delta_joint_cmds"
    servo_pose_cmds: str = "/servo_node/pose_cmds"
    servo_delta_twist_cmds: str = "/servo_node/delta_twist_cmds"
    servo_pause: str = "/servo_node/pause_servo"
    servo_switch_command_type: str = "/servo_node/switch_command_type"
    sim: bool = False

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
            trajectory_publisher="/arm_controller/joint_trajectory",
            action_type=ActionType.JOINT_POSITION, #ActionType.MOVEGROUP_FOLLOW_JOINT_TRAJECTION
            joint_trajectory_controller_sim="/joint_trajectory_controller/follow_joint_trajectory",
            joint_trajectory_controller="/passthrough_trajectory_controller/follow_joint_trajectory",
            joint_position_controller_commands="/forward_position_controller/commands",
            servo_delta_joint_cmds="/servo_node/delta_joint_cmds",
            servo_pose_cmds="/servo_node/pose_cmds",
            servo_delta_twist_cmds="/servo_node/delta_twist_cmds",
            servo_pause="/servo_node/pause_servo",
            servo_switch_command_type="/servo_node/switch_command_type",
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

