# Project 2.4: Robot Learning E2

**Avans Hogeschool Breda — 30-06-2026**

## Opdracht

Een AI-model trainen dat objecten kan oppakken en verplaatsen.

## Hardware

- UFactory UR5-robot
- TOS Teachbot
- Robotiq-gripper
- Pc
- Drie OpenCV-camera's

## Opstarten

Start de onderdelen in vijf afzonderlijke terminals.

### 1. Teachbot starten

Controleer of de joint angles ongeveer overeenkomen met `-8, 106, -72, -25, 10, -7`.

```bash
ros2 launch teachbot_ros teachbot_rviz.launch.py
```

#### Teachbot-informatie controleren

```bash
ping 192.168.1.50
ros2 topic echo /teachbot/joint_states
ros2 topic echo /teachbot/state
```

- `ping` controleert de verbinding met de Teachbot.
- `/teachbot/joint_states` toont de joint-state-informatie.
- `/teachbot/state` toont de status van de Teachbot-sensoren.

#### Teachbot starten zonder Teachbot Control-monitor

```bash
ros2 launch teachbot_ros teachbot_rviz.launch.py use_monitor_gui:=false
```

### 2. UR-robot starten

Start eerst de UR-robot via het paneel en verbind met het wifi-netwerk `TP-Link_84B8`.

```bash
ros2 launch my_ur_bringup real_robot.launch.py initial_joint_controller:=forward_position_controller
```

Controleer in de terminal of het proces succesvol is gestart. Druk daarna op **Start** op het paneel.

#### UR-robot-informatie controleren

```bash
ping 192.168.1.101
ros2 topic echo /joint_states
```

- `ping` controleert de verbinding met de UR-robot.
- `/joint_states` toont de joint-state-informatie.

### 3. MoveGroup starten

```bash
ros2 launch my_ur_bringup movegroup.launch.py launch_servo:=true
```

### 4. Robotiq-gripper starten

Controleer of de gripper opent en sluit. Voer het commando zo nodig opnieuw uit.

```bash
ros2 launch my_robotiq_controller robotiq_controller.launch.py
```

#### Gripper handmatig openen en sluiten

```bash
ros2 run robotiq_app control_gripper --open
ros2 run robotiq_app control_gripper --close
```

#### Gripper-informatie controleren

```bash
ros2 topic echo /robotiq_gripper_controller/joint_states
```

Dit topic toont de joint-state-informatie.

### 5. Teleoperation starten

Ga naar de LeRobot-omgeving en activeer de virtuele omgeving:

```bash
cd ~/lerobot
source .venv/bin/activate
```

#### Alleen met de Teachbot bewegen

```bash
lerobot-teleoperate \
  --robot.type=lerobot_robot_ur \
  --teleop.type=lerobot_teleoperator_teachbot \
  --fps=75
```

#### Actietype kiezen

Het actietype bepaalt hoe de UR-robot de commando's verwerkt. De standaardwaarde in `UrROS2InterfaceConfig` is `movegroup_servo_jog`. Je kunt de waarde aanpassen in `lerobot-robot-ur/lerobot_robot_ur/config_ur.py` of als optie meegeven aan `lerobot-teleoperate`:

```bash
lerobot-teleoperate \
  --robot.type=lerobot_robot_ur \
  --robot.ros2_interface.action_type=movegroup_servo_twist \
  --teleop.type=gamepad_6dof \
  --fps=15
```

Gebruik voor `movegroup_servo_twist` een teleoperator die lineaire en hoek-snelheden aanlevert. De Teachbot-teleoperator levert joint-posities aan en is daarom niet compatibel met deze twist-modus. Gebruik voor de Teachbot de standaardmodus `movegroup_servo_jog`.

De beschikbare waarden zijn:

| Waarde | Beschrijving | Status |
| --- | --- | --- |
| `movegroup_servo_jog` | Beweegt via MoveIt Servo met joint-jog-commando's. | Standaard; geschikt voor de Teachbot. |
| `movegroup_servo_twist` | Beweegt via MoveIt Servo met lineaire en hoek-snelheden. | Beschikbaar; vereist een teleoperator die twist-commando's levert. |
| `movegroup_follow_joint_trajectory` | Stuurt joint-trajecten naar de trajectory-controller. | Beschikbaar. |
| `joint_position` | Stuurt joint-posities naar de position-controller. | Beschikbaar. |
| `movegroup_servo_pose` | Bedoeld voor MoveIt Servo pose-commando's. | Gedeeltelijk; de UR-actieafhandeling is nog niet geïmplementeerd. |
| `cartesian_velocity` | Bedoeld voor Cartesian velocity-commando's. | Niet geïmplementeerd. |
| `none` | Selecteert geen ROS-control-backend. | Niet bruikbaar voor teleoperatie. |


## Dataset maken

1. Controleer het cameratype met `lerobot-find-cameras`.
2. Controleer of het camerabeeld goed is.
3. Vervang `XXX` in `--dataset.repo_id=zjandaman/XXX` door de gewenste datasetnaam.

```bash
lerobot-record \
  --robot.type=lerobot_robot_ur \
  --robot.id=Hughie \
  --teleop.type=lerobot_teleoperator_teachbot \
  --teleop.id=TeachHughie \
  --robot.cameras="
  {side: {type: opencv, index_or_path: 3, width: 640, height: 480, fps: 30},
  wrist: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30},
  top: {type: opencv, index_or_path: 6, width: 640, height: 480, fps: 30}}" \
  --display_data=true \
  --dataset.fps 15 \
  --dataset.repo_id=zjandaman/XXX \
  --dataset.num_episodes=30 \
  --dataset.single_task="Grab banana and deposit it in the box" \
  --dataset.streaming_encoding=true
```

## Dataset-episode afspelen

Episode `0` is de eerste episode. Vervang `zjandaman/Hail_Mary` door de gewenste dataset.

```bash
lerobot-replay \
  --robot.type=lerobot_robot_ur \
  --robot.id=Hughie \
  --dataset.repo_id=zjandaman/Hail_Mary \
  --dataset.episode=0
```

## Dataset trainen

Vervang `XXX` in `--policy.repo_id`, `--dataset.repo_id` en `--output_dir` door de gewenste naam. Bijvoorbeeld: `--policy.repo_id=zjandaman/x`, `--dataset.repo_id=zjandaman/x` en `--output_dir=outputs/train/x`.

```bash
lerobot-train \
  --policy.type=smolvla \
  --policy.device=cuda \
  --policy.repo_id=zjandaman/XXX \
  --dataset.repo_id=zjandaman/XXX \
  --batch_size=16 \
  --steps=20000 \
  --output_dir=outputs/train/XXX \
  --job_name=my_smolvla_training \
  --wandb.enable=false
```

## AI-model evalueren

Vervang `XXX` in `--dataset.repo_id=zjandaman/eval_XXX` en `--dataset.num_episodes=XXX`. Controleer ook dat `--policy.path` verwijst naar de getrainde policy.

```bash
lerobot-record \
  --robot.type=lerobot_robot_ur \
  --teleop.type=lerobot_teleoperator_teachbot \
  --robot.id=Hughie \
  --robot.cameras="
  {side: {type: opencv, index_or_path: 3, width: 640, height: 480, fps: 30},
  wrist: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30},
  top: {type: opencv, index_or_path: 6, width: 640, height: 480, fps: 30}}" \
  --dataset.single_task="Grab banana and deposit it in the box" \
  --display_data=true \
  --dataset.repo_id=zjandaman/eval_XXX \
  --dataset.episode_time_s=50 \
  --dataset.num_episodes=XXX \
  --policy.path=zjandaman/smolvlaXXX
```

## Laatst gemaakte SmolVLA-model uitvoeren

```bash
rm -rf /tmp/smolvla_test
lerobot-record \
  --robot.type=lerobot_robot_ur \
  --robot.id=Hughie \
  --teleop.type=lerobot_teleoperator_teachbot \
  --teleop.id=TeachHughie \
  --robot.cameras="
  {side: {type: opencv, index_or_path: 3, width: 640, height: 480, fps: 30},
  wrist: {type: opencv, index_or_path: 4, width: 640, height: 480, fps: 30},
  top: {type: opencv, index_or_path: 1, width: 640, height: 480, fps: 30}}" \
  --dataset.repo_id=debug/eval_no_save_test \
  --dataset.root=/tmp/smolvla_test \
  --dataset.single_task="Grab banana and deposit it in the box" \
  --dataset.episode_time_s=50 \
  --dataset.num_episodes=30 \
  --dataset.fps 15 \
  --dataset.push_to_hub=false \
  --policy.path=zjandaman/smolvla5 \
  --display_data=true
```

## ROS 2-communicatie

De volgende topics zijn ter informatie; voer ze niet als terminalcommando's uit.

### Gebruikte topics

- `/joint_states`
- `/robotiq_gripper_controller/joint_states`
- `/robotiq_gripper_controller/set_joint_state`
- `/teachbot/joint_states`
- `/teachbot/state`
