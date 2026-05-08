# MoveGroup Follow Joint Trajectory

:::{note}
Selecteer in de `class UrROS2InterfaceConfig(ROS2InterfaceConfig)` in het `config_ur.py` bestand de ` action_type: ActionType = ActionType.MOVEGROUP_FOLLOW_JOINT_TRAJECTORY` om deze functionaliteit te gebruiken.
:::

## Real Robot
### Start Teachbot
```bash
ros2 launch teachbot_ros teachbot_rviz.launch.py
```
### Start UR Robot
```bash
ros2 launch my_ur_bringup real_robot.launch.py 
```
### Start MoveGroup
```bash
ros2 launch my_ur_bringup move_group.launch.py 
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
ros2 launch my_ur_bringup simulation.launch.py
```

### Start teleoperatie
```bash
lerobot-teleoperate \
  --robot.type=lerobot_robot_ur \
  --teleop.type=lerobot_teleoperator_teachbot \
  --fps=15
  --robot.ros2_interface.sim=true
```
:::{note}
Het starten van de teleopratie kan alleen in de virtuele python omgeving van lerobot. Je kunt deze activeren door in een nieuwe terminal te navigeren naar de lerobot directory en het volgende commando uit te voeren:
```bash
cd ~/lerobot
source .venv/bin/activate
```
:::