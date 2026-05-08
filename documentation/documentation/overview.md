# Inleiding

Welkom bij de template voor het maken van `Lerobot ROS` projecten. 

Deze repository biedt een ROS2-integratie voor het Deze repository biedt een ROS2-integratie voor het LeRobot-framework, gericht op de Universal Robots (UR) robotarmen en de Teachbot. Hiermee kunnen gebruikers hun UR-robots teleopereren en datasets opnemen met behulp van ROS2-tools en -controllers.

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
