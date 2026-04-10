# LeRobot ROS

Deze repository biedt een ROS2-integratie voor het LeRobot-framework, gericht op de Universal Robots (UR) robotarmen en de Teachbot. Hiermee kunnen gebruikers hun UR-robots teleopereren en datasets opnemen met behulp van ROS2-tools en -controllers.


## Vereisten

### Softwarevereisten

Zorg ervoor dat het volgende is geïnstalleerd voordat u begint:

- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html) - Deze repo is alleen getest op Jazzy.
- [ros2_control](https://control.ros.org/rolling/index.html)
- [Universal Robots Template](https://avansmechatronica.github.io/my_ur_ROS2/)
- [Teachbot ROS software](https://avansmechatronica.github.io/teachbot/)

## Snelstart

Stel eerst LeRobot en lerobot-ros in een virtuele omgeving in. Let op: de Python-versie van de virtual environment moet compatibel zijn met uw ROS-versie. Voor ROS 2 Jazzy gebruiken we Python 3.12.

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


Ten slotte, om een teleoperatie uit te voeren:

```bash
# In terminal 1, start de Gazebo-simulatie van de UR-robot
ros2 launch my_ur_bringup simulation.launch.py

# In terminal 2, Laad de techbot ROS-software en start de ROS-node(simulatie)
ros2 launch teachbot_ros sim_teachbot_rviz.launch.py

# In terminal 3, start lerobot met de ROS-versie
lerobot-teleoperate \
  --robot.type=lerobot_robot_ur \
  --teleop.type=lerobot_teleoperator_teachbot \
  --fps=15
  --robot.ros2_interface.sim=true

```
Zodra teleoperatie werkt, kunt u alle standaard LeRobot-functies zoals gewoonlijk gebruiken.

## Activeren van environment
Bij het openen van een nieuwe terminal, navigeer naar de lerobot directory en activeer de virtuele omgeving:

```bash
cd ~/lerobot
source .venv/bin/activate
```

### Automatiseren van environment activatie
Niet aanbevolen kan conflicteren met andere ROS-omgevingen, maar als u wilt dat de lerobot virtuele omgeving automatisch wordt geactiveerd bij het openen van een nieuwe terminal, kunt u de volgende regel toevoegen aan uw shell-configuratiebestand (`~/.bashrc`):


```bash
# Automatisch activeren van de lerobot virtuele omgeving
if [ -d "$HOME/lerobot/.venv" ]; then
    source "$HOME/lerobot/.venv/bin/activate"
fi
```
Voeg deze regel toe aan het einde van uw `~/.bashrc`-bestand en sla het op. De volgende keer dat u een nieuwe terminal opent, zal de lerobot virtuele omgeving automatisch worden geactiveerd.

## Recording

Om een dataset op te nemen, gebruikt u het volgende commando:

```bash
DATASET_ROOT=$HOME/lerobot_recordings
DATASET_REPO_ID=./record-test

lerobot-record \
    --robot.type=lerobot_robot_ur \
    --robot.cameras="{ front: {type: opencv, index_or_path: /dev/video0, width: 640, height: 480, fps: 30}}" \
    --dataset.num_episodes=5 \
    --dataset.episode_time_s=15 \
    --dataset.single_task="Grab the black cube" \
    --dataset.streaming_encoding=true \
    --teleop.type=lerobot_teleoperator_teachbot \
    --dataset.vcodec=h264 \
    --dataset.push_to_hub=False \
    --dataset.repo_id=$DATASET_REPO_ID \
    --dataset.root=$DATASET_ROOT \
    --display_data=false \
    --dataset.encoder_threads=4 \
    --robot.ros2_interface.sim=true
```

## Replaying
Om een opgenomen dataset af te spelen, gebruikt u het volgende commando:
```bash
DATASET_ROOT=$HOME/lerobot_recordings
DATASET_REPO_ID=./record-test


lerobot-replay \
    --robot.type=lerobot_robot_ur \
    --dataset.repo_id=$DATASET_REPO_ID \
    --dataset.root=$DATASET_ROOT \
    --dataset.episode=0
```

## LeRobot Website
Voor meer informatie over het LeRobot-framework, bezoek de [LeRobot-website](https://huggingface.co/docs/lerobot/index).


## Referenties

- Gewrichtspositie met ros2_control
  - Met [joint_trajectory_controller](https://control.ros.org/rolling/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html)
  - Met [position_controllers](https://control.ros.org/rolling/doc/ros2_controllers/position_controllers/doc/userdoc.html)
- Grijper-besturing met ros2_control
  - Met [joint_trajectory_controller](https://control.ros.org/rolling/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html)
  - Met [Gripper Action Controller](https://control.ros.org/jazzy/doc/ros2_controllers/gripper_controllers/doc/userdoc.html)
