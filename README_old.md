# LeRobot ROS

Deze repository biedt een generieke ROS 2-interface voor het [LeRobot](https://github.com/huggingface/lerobot)-framework. Het fungeert als een lichtgewicht wrapper om elke [ros2_control](https://control.ros.org/rolling/index.html)- of [MoveIt](https://moveit.ai/)-compatibele robotarm te verbinden met het LeRobot-ecosysteem.

Er is ook een gamepad-teleoperator voor 6-DoF eindeffector-besturing en een toetsenbord-teleoperator voor gewrichtspositiebesturing meegeleverd.

**Ondersteunde besturingsmodi:**

- Gewrichtspositie met ros2_control
  - Met [joint_trajectory_controller](https://control.ros.org/rolling/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html)
  - Met [position_controllers](https://control.ros.org/rolling/doc/ros2_controllers/position_controllers/doc/userdoc.html)
- Eindeffector-snelheid met MoveIt 2
  - Met [MoveIt Servo](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html)
- Grijper-besturing met ros2_control
  - Met [joint_trajectory_controller](https://control.ros.org/rolling/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html)
  - Met [Gripper Action Controller](https://control.ros.org/jazzy/doc/ros2_controllers/gripper_controllers/doc/userdoc.html)

## Videodemo

[![lerobot-ros](https://markdown-videos-api.jorgenkh.no/url?url=https%3A%2F%2Fyoutu.be%2F8U8vDyi5IAs)](https://youtu.be/8U8vDyi5IAs)

## Vereisten

### Softwarevereisten

Zorg ervoor dat het volgende is geïnstalleerd voordat u begint:

- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html) - Deze repo is alleen getest op Jazzy.
- [ros2_control](https://control.ros.org/rolling/index.html)
- Als eindeffector-besturing gewenst is, moet [MoveIt2](https://moveit.ai/install-moveit2/binary) worden geïnstalleerd

## Snelstart met gesimuleerde SO-101

De onderstaande stappen stellen u in staat om toetsenbord-teleoperatie uit te voeren van een gesimuleerde SO-101-arm met behulp van Lerobot.

---

Stel eerst LeRobot en lerobot-ros in een virtuele omgeving in. Let op: de Python-versie van de virtualenv moet compatibel zijn met uw ROS-versie. Voor ROS 2 Jazzy gebruiken we Python 3.12.

```bash
# Maak en activeer een virtuele omgeving
conda create -y -n lerobot-ros python=3.12
conda activate lerobot-ros
conda install -c conda-forge libstdcxx-ng -y # nodig omdat rclpy GLIBCXX_3.4.30-symbolen vereist

# ROS-omgeving laden
source /opt/ros/jazzy/setup.sh

# lerobot-ros-pakketten installeren (dit installeert ook een compatibele versie van lerobot)
git clone https://github.com/ycheng517/lerobot-ros
cd lerobot-ros
pip install -e lerobot_robot_ros lerobot_teleoperator_devices
```

Stel vervolgens de gesimuleerde SO-101 in door de instructies te volgen op: https://github.com/Pavankv92/lerobot_ws

Ten slotte, om alle programma's uit te voeren:

```bash
# In terminal 1, start de Gazebo-simulatie
ros2 launch lerobot_description so101_gazebo.launch.py

# In terminal 2, laad de ros2-controllers en start MoveIt
ros2 launch lerobot_controller so101_controller.launch.py && \
  ros2 launch lerobot_moveit so101_moveit.launch.py

# In terminal 3, start lerobot met de ROS-versie van so101 en toetsenbord-teleop
cd <UW lerobot-ros DIRECTORY>
lerobot-teleoperate \
  --robot.type=so101_ros \
  --robot.id=my_awesome_follower_arm \
  --teleop.type=keyboard_joint \
  --teleop.id=my_awesome_leader_arm \
  --display_data=true
```

Zodra teleoperatie werkt, kunt u alle standaard LeRobot-functies zoals gewoonlijk gebruiken.

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

Deze optie wordt ingeschakeld door `action_type` in te stellen op `ActionType.MOVEGROUP_FOLLOW_JOINT_TRAJECTION` in de robotconfiguratie.

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