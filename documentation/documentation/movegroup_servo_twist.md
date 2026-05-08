# MoveGroup Servo Twist

:::{note}
Selecteer in de `class UrROS2InterfaceConfig(ROS2InterfaceConfig)` in het `config_ur.py` bestand de ` action_type: ActionType = ActionType.MOVEGROUP_SERVO_TWIST` om deze functionaliteit te gebruiken.
:::

## Real Robot
### Start Teachbot
```bash
ros2 launch teachbot_ros teachbot_rviz.launch.py
```
### Start UR Robot
```bash
ros2 launch my_ur_bringup real_robot.launch.py initial_joint_controller:=forward_position_controller
```
### Start MoveGroup
```bash
ros2 launch my_ur_bringup move_group.launch.py launch_servo:=true
```
### Start teleoperatie
```bash
lerobot-teleoperate \
  --robot.type=lerobot_robot_ur \
  --teleop.type=lerobot_teleoperator_teachbot \
  --fps=15
```
:::{note}
Het starten van de teleopratie kan alleen in de virtuele python omgeving van lerobot. Je kunt deze activeren door in een nieuwe terminal te navigeren naar de lerobot directory en het volgende commando uit te voeren:
```bash
cd ~/lerobot
source .venv/bin/activate
```
:::

## Simulation
### Start Teachbot
```bash
ros2 launch teachbot_ros sim_teachbot_rviz.launch.py
```

### Start UR Robot
```bash
ros2 launch my_ur_bringup simulation.launch.py initial_joint_controller:=forward_position_controller launch_servo:=true
```

### Start teleoperatie
```bash
lerobot-teleoperate \
  --robot.type=lerobot_robot_ur \
  --teleop.type=lerobot_teleoperator_teachbot \
  --fps=15
```
:::{note}
Het starten van de teleopratie kan alleen in de virtuele python omgeving van lerobot. Je kunt deze activeren door in een nieuwe terminal te navigeren naar de lerobot directory en het volgende commando uit te voeren:
```bash
cd ~/lerobot
source .venv/bin/activate
```
:::
