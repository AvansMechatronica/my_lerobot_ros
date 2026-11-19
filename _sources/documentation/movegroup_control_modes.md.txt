# MoveGroup-besturingsmodi

Deze pagina beschrijft de MoveGroup-besturingsmodi voor de UR-robot en hoe je
de gewenste modus instelt.

## Actietype kiezen

Het actietype bepaalt hoe de UR-robot de commando's verwerkt. De standaardwaarde
in `UrROS2InterfaceConfig` is `movegroup_servo_jog`. Selecteer een modus met de
commandline-optie `--robot.ros2_interface.action_type` of wijzig `action_type`
in `UrROS2InterfaceConfig` in
`lerobot-robot-ur/lerobot_robot_ur/config_ur.py`.

Bij LeLab stel je `action_type` in `config_ur.py` in; de LeLab-interface
biedt hiervoor geen commandline-optie. Start LeLab met `leLab` en selecteer
**Universal Robot (ROS 2)** en **Teachbot (ROS 2)** bij de apparaatinstellingen.
Deze combinatie is geschikt voor de joint-position-, Servo Jog- en Follow Joint
Trajectory-modi. Start LeLab vanuit dezelfde virtuele omgeving waarin de ROS
integraties zijn geïnstalleerd.

| Waarde | Beschrijving | Status |
| --- | --- | --- |
| `movegroup_servo_jog` | MoveIt Servo met joint-jog-commando's. | Standaard; geschikt voor de Teachbot. |
| `movegroup_servo_twist` | MoveIt Servo met lineaire en hoek-snelheden. | Vereist een teleoperator die twist-commando's levert. |
| `movegroup_follow_joint_trajectory` | Stuurt joint-trajecten naar de trajectory-controller. | Beschikbaar. |
| `joint_position` | Stuurt joint-posities naar de position-controller. | Beschikbaar. |
| `movegroup_servo_pose` | MoveIt Servo pose-commando's. | De UR-actieafhandeling is nog niet geïmplementeerd. |
| `cartesian_velocity` | Cartesian velocity-commando's. | Niet geïmplementeerd. |
| `none` | Selecteert geen ROS-control-backend. | Niet bruikbaar voor teleoperatie. |

## Virtuele omgeving activeren

Voer LeRobot- en LeLab-commando's uit vanuit de virtuele omgeving waarin LeLab
en de ROS-integraties zijn geïnstalleerd. In de installatievoorbeelden op deze
site is dat:

```bash
cd ~/lerobot
source .venv/bin/activate
```

## LeRobot of LeLab starten

Start eerst de ROS 2-drivers die bij de gekozen besturingsmodus horen in
afzonderlijke terminals. Zorg dat ROS 2 Jazzy en eventuele ROS-workspace
overlays in de omgeving van die terminals zijn gesourced. Activeer in de
terminal voor LeRobot of LeLab de virtuele omgeving zoals hierboven beschreven.

:::::{card}
::::{tab-set}

:::{tab-item} LeRobot
Start de teleoperatie met het commando uit de sectie van de gewenste
besturingsmodus. Bijvoorbeeld voor de standaardmodus Servo Jog:
```bash
lerobot-teleoperate \
  --robot.type=lerobot_robot_ur \
  --robot.ros2_interface.action_type=movegroup_servo_jog \
  --teleop.type=lerobot_teleoperator_teachbot \
  --fps=15
```

:::

:::{tab-item} LeLab
Start LeLab:
```bash
lelab
```
Open de apparaatinstellingen en selecteer **Universal Robot (ROS 2)** als robot
en **Teachbot (ROS 2)** als teleoperator. Start daarna de teleoperatie vanuit de
interface. Stel voor de UR de gewenste `action_type` vooraf in
`lerobot-robot-ur/lerobot_robot_ur/config_ur.py`; LeLab biedt hiervoor geen
commandline-optie. Als LeLab in een andere virtuele omgeving is geïnstalleerd,
activeer dan die omgeving in plaats van `~/lerobot/.venv`.
:::

::::
:::::

## MoveGroup Servo Jog

Servo Jog stuurt joint-jog-commando's naar MoveIt Servo. Dit is de
standaardmodus en de modus voor gebruik met de Teachbot.

### Echte robot

Start de Teachbot:

```bash
ros2 launch teachbot_ros teachbot_rviz.launch.py
```

Start de UR-robot:

```bash
ros2 launch my_ur_bringup real_robot.launch.py initial_joint_controller:=forward_position_controller
```

Start MoveGroup met Servo:

```bash
ros2 launch my_ur_bringup movegroup.launch.py launch_servo:=true
```

Start de teleoperatie:

:::::{card}
::::{tab-set}

:::{tab-item} LeRobot
```bash
lerobot-teleoperate \
  --robot.type=lerobot_robot_ur \
  --robot.ros2_interface.action_type=movegroup_servo_jog \
  --teleop.type=lerobot_teleoperator_teachbot \
  --fps=15
```
:::

:::{tab-item} LeLab
```bash
leLab
```

Start `leLab`, selecteer **Universal Robot (ROS 2)** en **Teachbot (ROS 2)**
en start de teleoperatie in de interface. Stel `action_type` vooraf in
`config_ur.py` in.
:::

::::
:::::

### Simulatie

Start de Teachbot-simulatie:

```bash
ros2 launch teachbot_ros sim_teachbot_rviz.launch.py
```

Start de UR-simulatie met MoveGroup Servo:

```bash
ros2 launch my_ur_bringup simulation.launch.py initial_joint_controller:=forward_position_controller launch_servo:=true
```

Start de teleoperatie:

:::::{card}
::::{tab-set}

:::{tab-item} LeRobot
```bash
lerobot-teleoperate \
  --robot.type=lerobot_robot_ur \
  --robot.ros2_interface.action_type=movegroup_servo_jog \
  --robot.ros2_interface.sim=true \
  --teleop.type=lerobot_teleoperator_teachbot \
  --fps=15
```
:::

:::{tab-item} LeLab
```bash
leLab
```
Start `leLab`, selecteer **Universal Robot (ROS 2)** en **Teachbot (ROS 2)**
en start de teleoperatie in de interface. Stel `action_type` vooraf in
`config_ur.py` in.
:::

::::
:::::

## MoveGroup Servo Twist

Servo Twist stuurt de robot aan met lineaire en hoek-snelheden. Gebruik een
teleoperator die twist-commando's levert, zoals `gamepad_6dof`. De
Teachbot-teleoperator levert joint-posities en is niet compatibel met deze
modus. LeLab's ingebouwde keyboard- en gamepad-teleoperators leveren
joint-jogcommando's, geen twist-commando's; gebruik hiervoor LeRobot met een
compatibele twist-teleoperator.

Verbind de fysieke controller (bijvoorbeeld via Bluetooth) met de computer
voordat je teleoperatie start.

### Echte robot

Start de UR-robot:

```bash
ros2 launch my_ur_bringup real_robot.launch.py initial_joint_controller:=forward_position_controller
```

Start MoveGroup met Servo:

```bash
ros2 launch my_ur_bringup movegroup.launch.py launch_servo:=true
```

Start de teleoperatie:

:::::{card}
::::{tab-set}

:::{tab-item} LeRobot
```bash
lerobot-teleoperate \
  --robot.type=lerobot_robot_ur \
  --robot.ros2_interface.action_type=movegroup_servo_twist \
  --teleop.type=gamepad_6dof \
  --fps=15
```
:::

:::{tab-item} LeLab
```bash
leLab
```
:::

::::
:::::

### Simulatie

Start de UR-simulatie met MoveGroup Servo:

```bash
ros2 launch my_ur_bringup simulation.launch.py initial_joint_controller:=forward_position_controller launch_servo:=true
```

Start de teleoperatie:

:::::{card}
::::{tab-set}

:::{tab-item} LeRobot
```bash
lerobot-teleoperate \
  --robot.type=lerobot_robot_ur \
  --robot.ros2_interface.action_type=movegroup_servo_twist \
  --teleop.type=gamepad_6dof \
  --fps=15
```
:::

:::{tab-item} LeLab
```bash
leLab
```

:::

::::
:::::

## MoveGroup Follow Joint Trajectory

Deze modus stuurt joint-trajecten naar de trajectory-controller. Gebruik de
Teachbot-teleoperator om de UR-robot te bedienen.

### Echte robot

Start de Teachbot:

```bash
ros2 launch teachbot_ros teachbot_rviz.launch.py
```

Start de UR-robot:

```bash
ros2 launch my_ur_bringup real_robot.launch.py
```

Start MoveGroup:

```bash
ros2 launch my_ur_bringup move_group.launch.py
```

Start de teleoperatie:

:::::{card}
::::{tab-set}

:::{tab-item} LeRobot
```bash
lerobot-teleoperate \
  --robot.type=lerobot_robot_ur \
  --robot.ros2_interface.action_type=movegroup_follow_joint_trajectory \
  --teleop.type=lerobot_teleoperator_teachbot \
  --fps=15
```
:::

:::{tab-item} LeLab
Start `leLab`, selecteer **Universal Robot (ROS 2)** en **Teachbot (ROS 2)**
en start de teleoperatie in de interface. Stel `action_type` vooraf in
`config_ur.py` in.
:::

::::
:::::

### Simulatie

Start de Teachbot-simulatie:

```bash
ros2 launch teachbot_ros sim_teachbot_rviz.launch.py
```

Start de UR-simulatie:

```bash
ros2 launch my_ur_bringup simulation.launch.py
```

Start de teleoperatie:

:::::{card}
::::{tab-set}

:::{tab-item} LeRobot
```bash
lerobot-teleoperate \
  --robot.type=lerobot_robot_ur \
  --robot.ros2_interface.action_type=movegroup_follow_joint_trajectory \
  --robot.ros2_interface.sim=true \
  --teleop.type=lerobot_teleoperator_teachbot \
  --fps=15
```
:::

:::{tab-item} LeLab
Start `leLab`, selecteer **Universal Robot (ROS 2)** en **Teachbot (ROS 2)**
en start de teleoperatie in de interface. Stel `action_type` vooraf in
`config_ur.py` in.
:::

::::
:::::

## MoveGroup Servo Pose

Servo Pose is niet geïmplementeerd in de UR-actieafhandeling. De huidige
teleoperatiecommando's sturen geen pose-commando's aan; selecteer deze modus
daarom niet voor teleoperatie.
