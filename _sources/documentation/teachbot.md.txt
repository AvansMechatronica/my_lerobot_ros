# class Teachbot(Teleoperator)


## get_action()
In de `get_action` methode worden de huidige joint states van de Teachbot gelezen en omgezet in een actie dictionary die de gewenste bewegingen en gripper positie bevat. De joint states worden gemapt naar de juiste formaten voor de verschillende controllers, afhankelijk van de configuratie van de `Teachbot`. De lineaire en hoekige snelheden worden bepaald door de huidige positie van de joints, terwijl de gripper positie wordt bepaald door de `gripper` joint state.
```
{'shoulder_pan_joint': 0.0, 'shoulder_lift_joint': 0.0, 'elbow_joint': 0.0, 'wrist_1_joint': 0.0, 'wrist_2_joint': 0.0, 'wrist_3_joint': 0.0, 'gripper.pos': 0.0}
```
:::{note}
Als in de configuratie `use_gripper` is ingesteld op `True` geld het volgende 
`vacuum_gripper:=True`: dan zal de `gripper.pos` waarde 0.0 zijn wanneer het vacuum uit is en 1.0 wanneer het vacuum aan is. 
`vacuum_gripper:=False`: dan zal de `gripper.pos` waarde 0.0 zijn wanneer de gripper open is en 100.0 wanneer deze geheel gesloten is.
:::

## Configuratie
De class `Teachbot(Teleoperator)` kan worden geconfigureerd in het `config_teachbot.py`. Pas daarvoor de `class TeachbotConfig(TeleoperatorConfig)` aan.