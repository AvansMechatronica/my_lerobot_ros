from types import SimpleNamespace

from lerobot_robot_ur.config_ur import UrConfig
from lerobot_robot_ur.ros_interface_ur import ROS2Interface


def test_joint_state_callback_merges_arm_and_gripper_messages():
    interface = ROS2Interface(UrConfig())
    interface._joint_state_callback(
        SimpleNamespace(name=["robotiq_85_left_knuckle_joint"], position=[0.35])
    )
    interface._joint_state_callback(
        SimpleNamespace(name=["shoulder_pan_joint"], position=[1.2])
    )

    assert interface.joint_state["position"]["gripper_joint"] == 0.35
    assert interface.joint_state["position"]["shoulder_pan_joint"] == 1.2


def test_joint_state_callback_ignores_missing_gripper_position():
    interface = ROS2Interface(UrConfig())
    interface._joint_state_callback(
        SimpleNamespace(name=["robotiq_85_left_knuckle_joint"], position=[])
    )

    assert "gripper_joint" not in interface.joint_state["position"]
