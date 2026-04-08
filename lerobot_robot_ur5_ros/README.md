# LeRobot plugin for Universal Robots UR5 with ROS2

This package provides the `ur5_ros` robot type for LeRobot, following the official plugin conventions.

## Installation

From the root of this package:

```bash
pip install -e .
```

## Usage

You can now use your robot with LeRobot CLI:

```bash
lerobot-teleoperate \
  --robot.type=ur5_ros \
  --robot.id=my_ur5_follower_arm \
  --teleop.type=teachbot_joint_state \
  --teleop.id=my_teachbot_leader_arm \
  --display_data=true
```
