# RemoteRobot

Futószalagon érkező termékek félautomata selejtdetekciója robotkar és VR eszközök segítségével, manuális vezérlési lehetőséggel.

## Használt technológiák:

- ROS kinetic
- MoveIt
- OpenCV
- Unity
- HTC Vive
- RabbitMQ

## Telepítés

UR5 robotkar MoveIt konfigurációs repo-ja
```bash
git clone https://github.com/fmauch/universal_robot.git src/universal_robot
```

RabbitMQ szerver létrehozása és indítása Docker-en a kommunikációhoz
```bash
docker run -d --hostname remote-robot-rabbit --name remoteRobotRabbit -p 8080:15672 rabbitmq:3-management
```

RabbitMQ *(AMQP)* kommunikációhoz szükséges Python csomag telepítése
```bash
apt-get install python-pip
python -m pip install pika --upgrade
```
