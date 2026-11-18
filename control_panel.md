

# ROS 2 Launch Manager

Start de grafische launcher vanuit een terminal waarin ROS 2 en de workspace
zijn gesourced:

```bash
python3 control_panel/launch_manager.py
```

Vul het ROS 2-pakket, het launch-bestand en eventuele extra launch-argumenten
in. Bijvoorbeeld:

| Pakket | Launch-bestand | Extra argumenten | RViz-argument |
| --- | --- | --- | --- |
| `teachbot_ros` | `teachbot_rviz.launch.py` |  | `rviz` |
| `my_ur_bringup` | `movegroup.launch.py` | `launch_servo:=true` | `launch_rviz` |

Met **RViz inschakelen** bepaalt u of het ingestelde RViz-argument op `true`
of `false` wordt gezet. Gebruik **Stop** om het launch-proces en bijbehorende
processen netjes te stoppen. De app erft de ROS-omgeving van de terminal
waaruit deze is gestart.

### 1. Teachbot starten


Met RVIZ
```bash
ros2 launch teachbot_ros teachbot_rviz.launch.py
```
Zonder RVIZ
```bash
ros2 launch teachbot_ros teachbot_rviz.launch.py rviz:=false
```



### 2. UR-robot starten

Start eerst de UR-robot via het paneel en verbind met het wifi-netwerk `TP-Link_84B8`.

```bash
ros2 launch my_ur_bringup real_robot.launch.py initial_joint_controller:=forward_position_controller
```

Controleer in de terminal of het proces succesvol is gestart. Druk daarna op **Start** op het paneel.


### 3. MoveGroup starten

Met RVIZ

```bash
ros2 launch my_ur_bringup movegroup.launch.py launch_servo:=true
```

Zonder RVIZ

```bash
ros2 launch my_ur_bringup movegroup.launch.py launch_servo:=true launch_rviz:=false
```

### 4. Robotiq-gripper starten


```bash
ros2 launch my_robotiq_controller robotiq_controller.launch.py
```

