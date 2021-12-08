# RemoteRobot

Futószalagon érkező termékek félautomata selejtdetekciója robotkar és VR eszközök segítségével, manuális vezérlési lehetőséggel.

## Használt technológiák:

- ROS kinetic
- MoveIt
- OpenCV
- Unity
- RTP
- ffmpeg
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

RabbitMQ kommunikációhoz szükséges Python csomag telepítése
```bash
apt-get install python-pip
python -m pip install pika --upgrade
```


## Futtatás

Szimulátor és MoveIt config indítása

`
roslaunch ur_gazebo ur5.launch
roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true
`

RabbitMQ -> ROS üzenetfordító indítása

`
rosrun rabbit_communicator_p rabbit_to_ros_f_e.py
`

ROS-on belül a robot mozgatásáért felelős modul indítása

`
rosrun move_robot_p robot_mover_f_e.py
`

RTP stream indítása

`
ffmpeg -video_size 1280x720 -framerate 30 -f x11grab -i :0.0+70,30 -f rtp -sdp_file source.sdp "rtp://192.168.57.1:5004"
`


RTP teszt

`
ffmpeg -stream_loop -1 -re -i test_video_2.mp4 -an -c:v copy -f rtp -sdp_file source.sdp "rtp://192.168.57.1:5004"
`

## RabbitMQ topic-ok

A rendszer kétféle üzemmódban képes működni, egy a beállításokat elvégző személy számára hasznos kofigurációs mód, míg a másik az általános felhasználó által mindennaposan használt felhasználói mód. A két módban eltérő üzenettípusokkal kommunikál a kliensoldal és a ROS.

### Mindkét módban aktív csatornák

`/set-mode`
 : felhasználási mód beállítása

`/robot-status`
 : itt küldi a ROS folyamatosan a robotkar állapotát

### Konfigurációs mód üzenetcsatornái

`/move-robot`
 : egy *(x, y, z)* alakú vektor segítségével elküldheti a kliensoldali szoftver, hogy mennyivel kell arrébb mozdítani a robotkar fejét

`/set-joints`
 : a robotkar csuklóinak egy kívánt állapotát lehet itt elküldeni *(j1, j2, j3, j4, j5)* formában

`/save-pos`
 : ha beállítottuk a robotkart egy adott pozícióba, akkor azt el tudjuk menteni, mint kindulási pozíció (home), vagy selejtellenőrzéshez bemutató pozíció (showpos)

### Általános használat közbeni csatornák

`/evaluate-req`
 : Ezen a csatornán kéri a ROS oldal, hogy a felhasználó állapítsa meg, hogy megfelelő minőségű-e a mutatott termék, miután azt bemutatta a kamerába. A kérést ellátja egy azonosítóval is, melyet a válaszban el kell küldjön a kliens, így elkerülhető a különböző termékvizsgálatok keveredése.

`/evaluate-move`
 : Itt kérheti a kliens, hogy a robot mozgassa újabb pozícióba a terméket, hogy jobban szemügyre lehessen venni

`/evaluate-result`
 : Itt ad választ a minőségellenőrzési kérdésre a kliens