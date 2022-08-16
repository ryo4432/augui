# AUGUI

AUGUI is a GUI for AUV(Autonomous Underwater Vehicle) on a web browser.

![](./doc/pic/demo.gif)

## feature

- display the auv Lat-Lon postion and direction on the map. (using the topic values of subscribed Odometry and NavSatFix type)
- display the time series graph of auv depth/altitude. (using the topic values of subscribed Odometry and Float64 type)
- display the hardware power on/off status. (using the topic value of subscribed Bool type)
- switch the hardware power on/off status. (using the response value of std_srvs/srv/SetBool type service)
- operation region for auv(blue rectangle). (using the response value of rcl_interfaces/srv/GetParameters type service)
- display the waypoint route. (using the response value of the waypoint information taking service)
- publish /cmd_vel from joystick controller value.

## environment

- ros2
  - confirmed foxy devel
  - dependencies
    - rosbrige_server: https://github.com/RobotWebTools/rosbridge_suite
- python 3.8.10
  - Tornado
  - scipy(for mockup node)
- others
  - jQuery
  - bootstrap
  - leaflet
  - leaflet.rotatedMarker.js
    - https://github.com/bbecquet/Leaflet.RotatedMarker
  - Chart.js
  - quaternion
    - https://github.com/infusion/Quaternion.js/
  - proj4

## how to install

```bash
mkdir -p ~/augui_ws/src
cd ~/augui_ws/src
git clone https://github.com/ryo4432/augui.git
cd ~/augui_ws
colcon build --symlink-install --packages-select augui
source install/setup.bash
```

## how to run

Run the following command, and access to `http://<your-robot-address>:8888`. You can use augui on the web browser.

```bash
ros2 launch augui web_app_launch.py
```

If you run just trial use or debugging mode, run this command,

```bash
ros2 launch augui web_app_launch.py debug:=True
```  

If you want to run the robot system and this app separately, run below on the robot machine,

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

and, run below on the another machine,

```bash
ros2 run augui augui
```