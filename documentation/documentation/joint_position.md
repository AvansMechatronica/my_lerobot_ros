# Joint Position

:::{note}
Selecteer in `UrROS2InterfaceConfig` in `config_ur.py` de
`action_type: ActionType = ActionType.JOINT_POSITION` om deze functionaliteit te
gebruiken. Dit geldt ook voor LeLab.
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
**Deze actie vereist geen MoveGroup, dus deze stap kan worden overgeslagen.*
```bash
ros2 launch my_ur_bringup move_group.launch.py 
```
### Start teleoperatie
:::::{card} 

::::{tab-set}


:::{tab-item} Lerobot
```bash
lerobot-teleoperate \
  --robot.type=lerobot_robot_ur \
  --teleop.type=lerobot_teleoperator_teachbot \
  --fps=15
```
:::

:::{tab-item} LeLab
```bash
leLab
```
Selecteer in LeLab **Universal Robot (ROS 2)** en **Teachbot (ROS 2)** bij de apparaatinstellingen en start daarna de teleoperatie.
:::
::::
:::::

:::{note}
Start de teleoperatie vanuit de virtuele Python-omgeving waarin LeRobot of
LeLab en de ROS-integraties zijn geïnstalleerd. Voor de installatie zoals
beschreven in deze documentatie:
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
ros2 launch my_ur_bringup simulation.launch.py initial_joint_controller:=forward_position_controller
```

### Start teleoperatie
:::::{card} 

::::{tab-set}


:::{tab-item} Lerobot
```bash
lerobot-teleoperate \
  --robot.type=lerobot_robot_ur \
  --teleop.type=lerobot_teleoperator_teachbot \
  --fps=15
```
:::

:::{tab-item} LeLab
```bash
leLab
```
Selecteer in LeLab **Universal Robot (ROS 2)** en **Teachbot (ROS 2)** bij de apparaatinstellingen en start daarna de teleoperatie.
:::
::::
:::::

:::{note}
Start de teleoperatie vanuit de virtuele Python-omgeving waarin LeRobot of
LeLab en de ROS-integraties zijn geïnstalleerd. Voor de installatie zoals
beschreven in deze documentatie:
```bash
cd ~/lerobot
source .venv/bin/activate
```
:::