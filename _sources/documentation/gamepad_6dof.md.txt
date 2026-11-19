# class Gamepad6DOFTeleop(Teleoperator)

Met deze class wordt een teleoperator geïmplementeerd die een 6DOF gamepad gebruikt om een UR robotarm te besturen. De gamepad biedt intuïtieve controle over zowel de lineaire als de hoekige bewegingen van de robot, evenals de gripper.

## 6DOF Gamepad controls
* Linker analog stick: Lineaire X/Y beweging
* Rechter analog stick: Hoekige X/Y rotatie (roll/pitch)
* LB/RB bumpers: Hoekige Z rotatie (yaw)
* LT/RT triggers: Lineaire Z beweging (omhoog/omlaag)
* A/X knop: Gripper controle (standaard open, sluit wanneer ingedrukt)

## get_action()
In de `get_action` methode worden de inputs van de gamepad gelezen en omgezet in een actie dictionary die de gewenste bewegingen en gripper positie bevat. De lineaire en hoekige snelheden worden bepaald door de positie van de sticks en triggers, terwijl de gripper positie wordt bepaald door de A/X knoppen.


```
{'linear_x.vel': np.float32(0.0), 'linear_y.vel': np.float32(0.0), 'linear_z.vel': np.float32(0.0), 'angular_x.vel': np.float32(0.0), 'angular_y.vel': np.float32(0.0), 'angular_z.vel': np.float32(0.0), 'gripper.pos': 0.0}
```
:::{note}
De `gripper.pos` waarde is 0.0 wanneer de gripper open is en 1.0 wanneer deze gesloten is.
:::

## Configuratie
De class `Gamepad6DOFTeleop(Teleoperator)`kan worden geconfigureerd in het `config_gamepad_6dof.py`. Pas daarvoor de `class Gamepad6DOFTeleopConfig(TeleoperatorConfig)` aan.