# Installatie van de Lerobot ROS Repository

## Vereisten

### Softwarevereisten

Zorg ervoor dat het volgende is geïnstalleerd voordat u begint:

- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html) - Deze repo is alleen getest op Jazzy.
- [ros2_control](https://control.ros.org/rolling/index.html)
- [Universal Robots Template](https://avansmechatronica.github.io/my_ur_ROS2/)
- [Teachbot ROS software](https://avansmechatronica.github.io/teachbot/)


## Installatie en setup Lerobot

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

```
## Installatie en setup Lerobot ROS integraties
:::::{card} 

::::{tab-set}


:::{tab-item} Zonder GIT-repository support

```bash
git clone https://github.com/AvansMechatronica/my_lerobot_ros.git
```

:::

:::{tab-item} Met GIT-repository support

* Maak een account aan bij [Github](https://github.com/) en login op dit account

* Open de [my_lerobot_ros](https://github.com/AvansMechatronica/my_lerobot_ros) repository

* Maak een Fork van de repository naar je eigen Github account door op het **Fork icoon**  te klikken:

![image](../images/fork.jpg)

* Volg de instructies, maar wijzig de naam van de nieuwe repository niet. Bevestig met **Create Fork**  

* Nu kun je de workspace als volgt creëren

```bash
git clone https://github.com/<jouw_account_naam>/my_lerobot_ros.git
```

*ps. Het gebruik van github (zoals add, commit & push commando's) valt  buiten de scope van deze documentatie*

:::

::::

:::::

### Installeer LeRobot specifieke ROS integraties
#### Installeer Teachbot support
```bash
cd ~/lerobot/my_lerobot_ros
pip install -e lerobot-teleoperator-teachbot
```

#### Installeer Universal Robot support
```bash
cd ~/lerobot/my_lerobot_ros
pip install -e lerobot-robot-ur

```
