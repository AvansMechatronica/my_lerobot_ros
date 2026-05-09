# Inleiding

Welkom bij de template voor het maken van `Lerobot ROS` projecten. 
Deze repository biedt een ROS2-integratie voor het LeRobot-framework, gericht op de Universal Robots (UR) robotarmen en de TOS Teachbot. Hiermee kunnen gebruikers hun UR-robots teleopereren en datasets opnemen met behulp van ROS2-tools en -controllers.

:::
{warning}
Deze documentatie is nog in ontwikkeling en kan onvolledig zijn.
Voor de meest recente informatie, zie de officiële documentatie van LeRobot: [LeRobot Documentatie](https://huggingface.co/docs/lerobot/index).
:::

## Officiële Lerobot Documentatie
Voor meer informatie over het LeRobot-framework, bezoek de officiële website: [LeRobot](https://huggingface.co/docs/lerobot/index).`

## Actiontypes en controllers
De volgende action types worden ondersteund in de `UrROS2Interface`. Deze zijn specefiek voor manipulator armen
action type | controller type | Opmerking
--- | --- | --- |
joint_position | forward_position_controller
movegroup_follow_joint_trajectory |scaled_joint_trajectory_controller | Voor real-robot
movegroup_follow_joint_trajectory |joint_trajectory_controller | Voor simulatie
movegroup_servo_twist | forward_position_controller | Voorkeur bij gebruik teleoprator welke een twist bericht genereert, zoals een joystick teleoperator
movegroup_servo_pose | forward_position_controller | Nog niet getest
movegroup_servo_jog | forward_position_controller | Voorkeur bij gebruik Teachbot teleoperator

## Lerobot taken
In deze documentatie wordt beschreven hoe een setup gemaakt kan worden voor een teleoperatie door een joystick of teachbot naar een UR robotarm. Als deze inegraties werken kunnen andere Lerobt taken ook gebruikt worden, zoals het opnemen van datasets.

Zie voor meer informatie over deze taken de officiële documentatie van Lerobot:
* [teleoperate with cameras](https://huggingface.co/docs/lerobot/il_robots#teleoperate-with-cameras)
* [record a dataset](https://huggingface.co/docs/lerobot/il_robots#record-a-dataset)
* [replay an episode](https://huggingface.co/docs/lerobot/il_robots#replay-an-episode)
* [run inference and evaluate your policy](https://huggingface.co/docs/lerobot/il_robots#run-inference-and-evaluate-your-policy)

    
## Dank aan Yifei Cheng
Deze ROS2-integratie is mede mogelijk gemaakt door de inspiratie van Yifei Cheng's werk aan [lerobot-ros](https://github.com/ycheng517/lerobot-ros)
