# LeRobot ROS

Deze repository biedt een generieke ROS 2-interface voor het [LeRobot](https://github.com/huggingface/lerobot)-framework. Het fungeert als een lichtgewicht wrapper om elke [ros2_control](https://control.ros.org/rolling/index.html)- of [MoveIt](https://moveit.ai/)-compatibele robotarm te verbinden met het LeRobot-ecosysteem.



**Ondersteunde besturingsmodi:**

- Gewrichtspositie met ros2_control
  - Met [joint_trajectory_controller](https://control.ros.org/rolling/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html)
  - Met [position_controllers](https://control.ros.org/rolling/doc/ros2_controllers/position_controllers/doc/userdoc.html)
- Eindeffector-snelheid met MoveIt 2
  - Met [MoveIt Servo](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html)
- Grijper-besturing met ros2_control
  - Met [joint_trajectory_controller](https://control.ros.org/rolling/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html)
  - Met [Gripper Action Controller](https://control.ros.org/jazzy/doc/ros2_controllers/gripper_controllers/doc/userdoc.html)


## Vereisten

### Softwarevereisten

Zorg ervoor dat het volgende is geïnstalleerd voordat u begint:

- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html) - Deze repo is alleen getest op Jazzy.
- [ros2_control](https://control.ros.org/rolling/index.html)
- [Universal Robots Template](https://avansmechatronica.github.io/my_ur_ROS2/)
- [Teachbot ROS software](https://avansmechatronica.github.io/teachbot/)

## Snelstart

Stel eerst LeRobot en lerobot-ros in een virtuele omgeving in. Let op: de Python-versie van de virtualenv moet compatibel zijn met uw ROS-versie. Voor ROS 2 Jazzy gebruiken we Python 3.12.

```bash
# Controleer dat Python 3.12 beschikbaar is voor ROS 2 Jazzy
python3 --version

# Maak een virtuele Python omgeving in de lerobot directory
mkdir ~/lerobot
cd ~/lerobot
python3 -m venv .venv
source .venv/bin/activate

# Installeer LeRobot software
git clone https://github.com/huggingface/lerobot.git
cd lerobot
pip install -e .

# Installeer LeRobot specifieke ROS integraties
cd ~/lerobot
git clone https://github.com/AvansMechatronica/my_lerobot_ros.git
cd my_lerobot_ros
# Installeer Teachbot support
pip install -e lerobot-teleoperator-teachbot
# Installeer Universal Robot support
pip install -e lerobot-robot-ur

```


Ten slotte, om alle programma's uit te voeren:

```bash
# In terminal 1, start de Gazebo-simulatie


# In terminal 2, laad de ros2-controllers en start MoveIt


# In terminal 3, start lerobot met de ROS-versie
lerobot-teleoperate \
  --robot.type=lerobot_robot_ur \
  --teleop.type=lerobot_teleoperator_teachbot \
  --fps=60

```
Zodra teleoperatie werkt, kunt u alle standaard LeRobot-functies zoals gewoonlijk gebruiken.

## Activeren van environment
Bij het openen van een nieuwe terminal, navigeer naar de lerobot directory en activeer de virtuele omgeving:

```bash
cd ~/lerobot
source .venv/bin/activate
```

### Automatiseren van environment activatie
Om het activeren van de virtuele omgeving te automatiseren bij het openen van een nieuwe terminal, kunt u de volgende regel toevoegen aan uw shell-configuratiebestand (~/.bashrc`):

```bash
# Automatisch activeren van de lerobot virtuele omgeving
if [ -d "$HOME/lerobot/.venv" ]; then
    source "$HOME/lerobot/.venv/bin/activate"
fi
```
Voeg deze regel toe aan het einde van uw `~/.bashrc`-bestand en sla het op. De volgende keer dat u een nieuwe terminal opent, zal de lerobot virtuele omgeving automatisch worden geactiveerd.









# Hieronder nog aanpassen




## Handleiding voor Robot-integratie

In dit gedeelte wordt beschreven hoe andere ROS-gebaseerde robots kunnen worden geïntegreerd met Lerobot.

### Besturingsmodi voor de Arm

Momenteel ondersteunt de repo de volgende besturingsmodi voor de arm:

**Optie 1: Gewrichtspositiebesturing**

Deze optie gebruikt [position_controllers](https://control.ros.org/rolling/doc/ros2_controllers/position_controllers/doc/userdoc.html) in `ros2_control`. De robot moet beschikken over:

- `position_controllers/JointGroupPositionController` voor de robotarmgewrichten
- `joint_state_broadcaster/JointStateBroadcaster` voor terugkoppeling van de gewrichtstoestand

Deze optie wordt ingeschakeld door `action_type` in te stellen op `ActionType.JOINT_POSITION` in de robotconfiguratie.

**Optie 2: Gewrichtsbaanbesturing**

Deze optie gebruikt [joint_trajectory_controller](https://control.ros.org/rolling/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html) in `ros2_control`. De robot moet beschikken over:

- `joint_trajectory_controller/JointTrajectoryController` voor de robotarmgewrichten
- `joint_state_broadcaster/JointStateBroadcaster` voor terugkoppeling van de gewrichtstoestand

Deze optie wordt ingeschakeld door `action_type` in te stellen op `ActionType.JOINT_TRAJECTORY` in de robotconfiguratie.

**Optie 3: Eindeffector-besturing**

Deze optie gebruikt [MoveIt Servo](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html) in MoveIt. De robot moet beschikken over:

- Het `moveit_servo`-knooppunt voor realtime eindeffector-besturing
- `joint_trajectory_controller/JointTrajectoryController` voor robotarmbesturing
- `joint_state_broadcaster/JointStateBroadcaster` voor terugkoppeling van de gewrichtstoestand

Deze optie wordt ingeschakeld door `action_type` in te stellen op `ActionType.CARTESIAN_VELOCITY` in de robotconfiguratie. Zie: [ar4_ros_driver](https://github.com/ycheng517/ar4_ros_driver) voor een voorbeeld van het gebruik van `moveit_servo`.

### Grijper-besturingsmodi

De repo ondersteunt twee grijper-besturingsmodi die kunnen worden geconfigureerd via de instelling `gripper_action_type`:

**Baanbesturing (`GripperActionType.TRAJECTORY`)**

- Gebruikt `JointTrajectoryController` van ros2_control
- Publiceert `JointTrajectory`-berichten naar `/gripper_controller/joint_trajectory`

**Actiebesturing (`GripperActionType.ACTION`)**

- Gebruikt `GripperActionController` van ros2_control
- Stuurt actiedoelen naar `/gripper_controller/gripper_cmd`
- Geeft terugkoppeling of de grijper de doelpositie heeft bereikt

### Codewijzigingen in Lerobot-ros

Breid de klasse `ROS2Robot` uit in [robot.py](./lerobot_robot_ros/lerobot_robot_ros/robot.py).
Deze klasse kan een eenvoudige doorvoer zijn. Ze is alleen nodig om te voldoen aan de vereisten voor apparaatdetectie van lerobot.

```python
class MyRobot(ROS2Robot):
  pass
```

Maak vervolgens een configuratieklasse voor uw robot door `ROS2Config` te subclassen in [config.py](./lerobot_robot_ros/lerobot_robot_ros/config.py).
De naam van deze klasse moet gelijk zijn aan de naam van uw robotklasse, gevolgd door `Config`.
U kunt gewrichtsnamen, grijperconfiguraties en andere parameters naar wens overschrijven.
Een voorbeeldconfiguratieklasse voor gewrichtssnelheidsbesturing kan er als volgt uitzien:

```python
from dataclasses import dataclass, field
from lerobot.common.robots.config import RobotConfig
from lerobot.common.robots.config import ROS2Config, ROS2InterfaceConfig

@RobotConfig.register_subclass("my_ros2_robot")
@dataclass
class MyRobotConfig(ROS2Config):
    action_type: ActionType = ActionType.JOINT_POSITION

    ros2_interface: ROS2InterfaceConfig = field(
        default_factory=lambda: ROS2InterfaceConfig(
            base_link="base_link",
            arm_joint_names=[
                "joint_1",
                "joint_2",
                "joint_3",
                "joint_4",
                "joint_5",
                "joint_6",
            ],
            gripper_joint_name="gripper_joint",
            gripper_open_position=0.0,
            gripper_close_position=1.0,
            max_linear_velocity=0.05,  # m/s
            max_angular_velocity=0.25,  # rad/s
        )
    )
```