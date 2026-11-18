# Launch Control Panel - Functies

## 🎨 Grafische Interface

```
┌─────────────────────────────────────────────────────────┐
│  ROS 2 Launch Control Panel                             │
│  Overzicht van launches - Klik op Start/Stop            │
├─────────────────────────────────────────────────────────┤
│                                                          │
│  ● Teachbot (met RViz)                 ▶ Start  ⏹ Stop │
│    Start Teachbot met RViz visualisatie                 │
│    $ ros2 launch teachbot_ros teachbot_rviz.launch.py  │
│    $ rviz:=true                                         │
│                                                          │
│  ● UR-robot                            ▶ Start  ⏹ Stop │
│    Start de UR-robot                                    │
│    $ ros2 launch my_ur_bringup real_robot.launch.py    │
│                                                          │
│  ● MoveGroup (met Servo)               ▶ Start  ⏹ Stop │
│    Start MoveGroup met Servo en RViz                    │
│    $ ros2 launch my_ur_bringup movegroup.launch.py     │
│                                                          │
│  ● Robotiq-gripper                     ▶ Start  ⏹ Stop │
│    Start de Robotiq-gripper controller                  │
│                                                          │
└─────────────────────────────────────────────────────────┘
```

## ✨ Voordelen

✅ **Overzichtelijk**: Alle launches in één paneel  
✅ **Eenvoudig**: Start/Stop met één klik  
✅ **Configureerbaar**: Pas launches aan in YAML  
✅ **Real-time logging**: Zien wat er gebeurt  
✅ **Status indicatoren**: Groen/oranje/rood status  
✅ **Netjes afsluiten**: Proper signal handling (SIGINT → SIGTERM → SIGKILL)  
✅ **Multi-launch**: Meerdere launches naast elkaar  

## 🚀 Hoe starten

```bash
# In een terminal met ROS 2 sourced:
python3 control_panel/launch_control_panel.py
```

## ⚙️ Launches toevoegen

Edit `control_panel/launches.yaml`:

```yaml
launches:
  - name: "Mijn nieuwe launch"
    package: "my_package"
    launch_file: "my_launch.launch.py"
    arguments: ""
    rviz_argument: ""
    rviz_enabled: false
    description: "Wat doet deze launch?"
```

Klaar! Herstart het paneel en je ziet de nieuwe launch.

## 📊 Status Indicators

| Kleur | Betekenis |
|-------|-----------|
| 🟢 Groen | Launch draait |
| 🟠 Oranje | Bezig met stoppen |
| 🔴 Rood | Gestopt |
| ⚫ Grijs | Niet gestart |

## 🛠️ Wat je kan doen

### Een launch starten
1. Klik op **▶ Start**
2. Output verschijnt in het log-venster
3. Status wordt groen

### Een launch stoppen
1. Klik op **⏹ Stop**
2. Status wordt oranje (stopt…)
3. Na een paar seconden: rood (gestopt)

### Meerdere launches tegelijk
- Je kan meerdere launches tegelijk laten draaien
- Elk heeft zijn eigen uitvoerlog

### Logs bekijken
- Scroll in het output-venster
- Ziet real-time wat ROS 2 doet

## 📝 Configuratie-voorbeelden

### Teachbot
```yaml
  - name: "Teachbot (met RViz)"
    package: "teachbot_ros"
    launch_file: "teachbot_rviz.launch.py"
    rviz_argument: "rviz"
    rviz_enabled: true
    description: "Start Teachbot met RViz"
```

### UR-robot + MoveGroup
```yaml
  - name: "UR-robot"
    package: "my_ur_bringup"
    launch_file: "real_robot.launch.py"
    arguments: "initial_joint_controller:=forward_position_controller"
    description: "Verbind eerst WiFi TP-Link_84B8"

  - name: "MoveGroup"
    package: "my_ur_bringup"
    launch_file: "movegroup.launch.py"
    arguments: "launch_servo:=true"
    rviz_argument: "launch_rviz"
    rviz_enabled: true
    description: "Start MoveGroup met Servo"
```

### Gripper
```yaml
  - name: "Robotiq-gripper"
    package: "my_robotiq_controller"
    launch_file: "robotiq_controller.launch.py"
    description: "Gripper controller"
```

## 🔗 Bestanden

- `launch_control_panel.py` - Hoofd-applicatie
- `launches.yaml` - Configuratie van launches
- `launch_manager.py` - Oudere versie (handmatig)
- `README.md` - Gedetailleerde documentatie
- `FEATURES.md` - Dit bestand

## 📌 Snelle commando's

```bash
# Start het panel
python3 control_panel/launch_control_panel.py

# Of als je in een andere map bent:
python3 /path/to/control_panel/launch_control_panel.py

# Check of PyYAML geïnstalleerd is:
python3 -c "import yaml; print('OK')"
```

---

**Veel plezier met je ROS 2 launches!** 🤖
