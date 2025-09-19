## How to access different PCs

1. OnLogic Industrial PC (KU Leuven)
```bash
ssh robpc@192.168.1.7
```
password: robpc

2. PC to control wheels of MAV (with docker image):
```bash
ssh root@10.10.6.100 -p 2223
```
password: ubuntu

```bash
cd /home/ubuntu/mav_robleix_ws
source install/setup.bash
ros2 run schwarzmuller_driver schwarzmuller_driver_node
ros2 run schwarzmuller_driver schwarzmuller_driver_lifecycle_node
```

3. PC to activate/deactivate Navitec/Navitrol:
```bash
ssh -p 22 root@10.10.6.100
```
password: maestro

```bash
maestro-vpn start #Activates VPN for Neura support to be able to access PC remotely
maestro-vpn strop #Deactivates VPN for Neura support to be able to access PC remotely
maestro-vpn #Checks the status of VPN
maestro-agv start #Starts Navitec/Navitrol software
maestro-agv stop #Starts Navitec/Navitrol software
```

4. PC with realtime Kernel and software to control Maira

```bash
ssh hrg@192.168.2.13
```
password: hrg

## RUN

 1. Action server for the routing application:

``` bash
robelix_source
cd etasl_ros2_application_template/applications/application_robelix/
python3 routing_action_server.py
```
2. Vision (Realsense + board localization) and F/T sensor node (Has the camera calibration result)
``` bash
robelix_source
ros2 launch board_localization localization_launch.py
``` 

3. GUI (FLASK + Action client GUI)

``` bash
robelix_source
cd neura-gui/
source gui_env/bin/activate
cd gui/
python3 server.py
```

4. Gripper launch file (Could be added to vision & F/T Sensor launch file)
``` bash
robelix_source
ros2 launch schunk_egu_egk_gripper_driver schunk.launch.py IP:=192.168.78.105
```

All the routing application parameters are in ```etasl_ros2_application_template/applications/application_robelix/parameters/application_params.json```

5. Launch the MAV schwarzmuller driver from :

First access the correct PC:
```bash
ssh root@10.10.6.100 -p 2223
```
password: ubuntu


And then run the driver. There are two versions:

- a regular ROS2 node:

```bash
source /home/ubuntu/mav_robleix_ws/install/setup.bash
export ROS_DOMAIN_ID=18
ros2 run schwarzmuller_driver schwarzmuller_driver_node
```

- and a Lifecycle node (done by KU Leuven):

```bash
source /home/ubuntu/mav_robleix_ws/install/setup.bash
export ROS_DOMAIN_ID=18
ros2 run schwarzmuller_driver schwarzmuller_driver_lifecycle_node
```

which can transition states by calling the following

```bash
source /home/ubuntu/mav_robleix_ws/install/setup.bash
export ROS_DOMAIN_ID=18
ros2 lifecycle set schwarzmuller_driver_lifecycle_node configure
ros2 lifecycle set schwarzmuller_driver_lifecycle_node activate #Release the brakes and waits for command through topic
ros2 lifecycle set schwarzmuller_driver_lifecycle_node deactivate #Activates the brakes
```
