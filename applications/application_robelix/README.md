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