# ROS 2 Launch Control Panel

## Overzicht

Dit is een geavanceerde grafische interface (GUI) voor het starten en stoppen van ROS 2 launch-processen. 
Alle launches zijn geconfigureerd in een YAML-bestand, zodat je gemakkelijk launches kan toevoegen, verwijderen 
of aanpassen zonder code te hoeven wijzigen.

## Snelle Start

Zorg dat ROS 2 en je workspace zijn gesourced in de terminal:

```bash
source /opt/ros/<distro>/setup.bash
source install/setup.bash
```

Start vervolgens het Launch Control Panel:

```bash
python3 control_panel/launch_control_panel.py
```

## Wat je ziet

- **Titel en beschrijving**: Voor elke launch
- **Status indicator**: Groen (draaiend), oranje (stopt…), rood (gestopt), grijs (niet gestart)
- **Start-knop**: Zet de launch aan
- **Stop-knop**: Stopt de launch netjes af
- **Output-logs**: Realtime output van het ROS 2-process

## Launches configureren

Bewerk `launches.yaml` om launches toe te voegen:

```yaml
launches:
  - name: "Teachbot (met RViz)"
    package: "teachbot_ros"
    launch_file: "teachbot_rviz.launch.py"
    arguments: ""
    rviz_argument: "rviz"
    rviz_enabled: true
    description: "Start Teachbot met RViz visualisatie"
```

### Configuratie-opties

| Optie | Type | Beschrijving | Voorbeeld |
|-------|------|-------------|----------|
| `name` | string | Weergavenaam in het panel | `"Teachbot (met RViz)"` |
| `package` | string | ROS 2 package-naam | `teachbot_ros` |
| `launch_file` | string | Naam van launch-bestand | `teachbot_rviz.launch.py` |
| `arguments` | string | Spatie-gescheiden argumenten | `launch_servo:=true servo_debug:=false` |
| `rviz_argument` | string | Naam van RViz-argument (leeg als niet gebruikt) | `rviz` of `launch_rviz` |
| `rviz_enabled` | boolean | Initiële RViz-status | `true` of `false` |
| `description` | string | Korte beschrijving (optioneel) | `"Start robot zonder sim"` |

## Voorbeelden

### Teachbot met/zonder RViz

```yaml
  - name: "Teachbot (met RViz)"
    package: "teachbot_ros"
    launch_file: "teachbot_rviz.launch.py"
    rviz_argument: "rviz"
    rviz_enabled: true
    description: "Teachbot met visualisatie"

  - name: "Teachbot (zonder RViz)"
    package: "teachbot_ros"
    launch_file: "teachbot_rviz.launch.py"
    rviz_argument: "rviz"
    rviz_enabled: false
    description: "Teachbot zonder visualisatie"
```

### UR-robot met MoveGroup

```yaml
  - name: "UR-robot"
    package: "my_ur_bringup"
    launch_file: "real_robot.launch.py"
    arguments: "initial_joint_controller:=forward_position_controller"
    description: "Verbind eerst met WiFi TP-Link_84B8"

  - name: "MoveGroup (met Servo)"
    package: "my_ur_bringup"
    launch_file: "movegroup.launch.py"
    arguments: "launch_servo:=true"
    rviz_argument: "launch_rviz"
    rviz_enabled: true
    description: "Start MoveGroup met Servo en RViz"
```

### Robotiq-gripper

```yaml
  - name: "Robotiq-gripper"
    package: "my_robotiq_controller"
    launch_file: "robotiq_controller.launch.py"
    description: "Robotiq gripper controller"
```

## Hoe het werkt

1. **YAML laden**: Het programma laadt alle launches uit `launches.yaml`
2. **UI bouwen**: Voor elke launch wordt een knop gemaakt met Start/Stop
3. **Starten**: Klikt op Start → `ros2 launch` command wordt uitgevoerd
4. **Output**: Realtime output wordt getoond in het log-venster
5. **Stoppen**: Klikt op Stop → Process wordt netjes afgesloten (SIGINT → SIGTERM → SIGKILL)

## Geavanceerde opties

### Meerder argumenten

Gebruik spaties om meerdere argumenten te scheiden:

```yaml
arguments: "arg1:=value1 arg2:=value2 arg3:=true"
```

### Geen RViz-argument

Laat `rviz_argument` leeg als de launch geen RViz-argument gebruikt:

```yaml
rviz_argument: ""
```

## Troubleshooting

### "Launches.yaml niet gevonden"
Zorg dat je `launches.yaml` in dezelfde map staat als `launch_control_panel.py`

### "ROS 2 starten mislukt"
- Controleer dat je ROS 2 en je workspace hebt gesourced
- Controleer dat het package en launch-bestand juist zijn gespeld
- Voer handmatig `ros2 launch <package> <file>` uit in de terminal

### Output is leeg
Wacht even, output kan vertraagd zijn. Check ook het `launch_manager.py` als je debugging nodig hebt.

## Verschil met launch_manager.py

| Functie | LaunchControlPanel | LaunchManager |
|---------|-------------------|---------------|
| Meerdere launches | ✓ | ✗ |
| Config-bestand | YAML | Handmatig invoeren |
| Status indicatoren | ✓ | ✓ |
| Output-logging | Per launch | Globaal |
| Gebruiksgemak | Hoog | Laag (eenmalig gebruik) |

## Tips

- **Wacht op output**: Sommige launches duren even voordat ze klaar zijn
- **Geen simultaan starten**: Start launches sequentieel om problemen te voorkomen
- **Check logs**: De logs helpen bij troubleshooting
- **Stop netjes**: Klikt altijd op Stop i.p.v. het venster forcibly sluiten

## Licentie

Dezelfde als het project
