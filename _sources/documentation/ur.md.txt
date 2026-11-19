# class Ur(Robot)


Met deze class wordt een UR robotarm in Lerobot geïmplementeerd die kan worden bestuurd via verschillende teleoperators, zoals de `Gamepad6DOFTeleop` of `Teachbot`. De UR robotarm is een veelzijdige en veelgebruikte robot in de industrie, bekend om zijn precisie en flexibiliteit.


## send_action() 
De `send_action` methode is verantwoordelijk voor het verzenden van de actie commando's naar de UR robotarm. Afhankelijk van het type controller dat wordt gebruikt (bijvoorbeeld `forward_position_controller`, `scaled_joint_trajectory_controller`, of `joint_trajectory_controller`), zal deze methode de actie commando's omzetten in het juiste formaat en deze verzenden via ROS2-interfaces. De actie commando's bevatten informatie over de gewenste lineaire en hoekige bewegingen van de robot, evenals de gripper positie.  

### Voorbeeld van een actie dictionary (Twist operatie):
```
{'linear_x.vel': np.float32(0.0), 'linear_y.vel': np.float32(0.0), 'linear_z.vel': np.float32(0.0), 'angular_x.vel': np.float32(0.0), 'angular_y.vel': np.float32(0.0), 'angular_z.vel': np.float32(0.0), 'gripper.pos': 0.0}
```
### Voorbeeld van een actie dictionary (Joint positie operatie):
```
{'shoulder_pan_joint': 0.0, 'shoulder_lift_joint': 0.0, 'elbow_joint': 0.0, 'wrist_1_joint': 0.0, 'wrist_2_joint': 0.0, 'wrist_3_joint': 0.0, 'gripper.pos': 0.0}
```

## Gripper controle
De gripper van de UR robotarm kan worden gecontroleerd door de `gripper.pos` waarde in de actie dictionary aan te passen. 

### Vacuum gripper controle:
Een waarde van `0.0` betekent dat er geen vacuum wordt toegepast, terwijl een waarde van `1.0` betekent dat er wel vacuum wordt toegepast. 

### Vinger gripper controle:
Een waarde van `0.0` betekent dat de gripper volledig open is, terwijl een waarde van `100.0` betekent dat de gripper volledig gesloten is.

:::{note}
**Gripper controller is nog niet geïmplementeerd en is aan Avans studenten om dit verder uit te werken.**
:::