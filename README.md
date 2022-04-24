# RemoteRobot

Futószalagon érkező termékek félautomata selejtdetekciója robotkar és VR eszközök segítségével, manuális vezérlési lehetőséggel.

## Használt technológiák:

- ROS noetic
- MoveIt
- OpenCV
- Unity
- RTP
- ffmpeg
- HTC Vive
- RabbitMQ

## Telepítés

ROS noetic telepítése:
http://wiki.ros.org/noetic/Installation/Ubuntu

MoveIt! telepítése ROS noetic-hez:
https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html

UR5 robotkar MoveIt konfigurációs repo-ja
```bash
git clone https://github.com/fmauch/universal_robot.git catkin_ws/src/universal_robot
```
vagy inkább
```bash
git clone -b kinetic-devel https://github.com/ros-industrial/universal_robot.git catkin_ws/src/universal_robot
```

Hiányzó ROS package-k telepítése
```bash
rosdep install -y --from-paths . --ignore-src --rosdistro noetic
```

RabbitMQ szerver és webes vezérlés indítása
```bash
docker-compose up
```

Szükséges Python csomagok telepítése
```bash
wget https://boostrap.pypa.io/pip/2.7/get-pip.py
python2 get-pip.py

pip2 install pika pyyaml enum rospkg
```


## Futtatás

Szimulátor és MoveIt config indítása

```bash
roslaunch ur_gazebo ur5.launch
roslaunch ur5_e_moveit_config ur5_e_moveit_planning_execution.launch sim:=true limited:=true
```

RViz
```bash
roslaunch ur5_e_moveit_config moveit_rviz.launch config:=true
```

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
