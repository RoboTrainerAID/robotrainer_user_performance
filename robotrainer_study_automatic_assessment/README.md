# User study automatic assessment

## Todos

- [ ] Soll alle topics kontrollieren ob gepublished wird
- [ ] Richtige Bluetooth adapter id kontrollieren
- [ ] Feedback geben welches topic fehlt oder freigabe für test geben
- [ ] Kontrollieren ob alle nodes laufen?
- [ ] launch files für alle nodes
- [ ] automatisch welchsel zwischen verschiedenen Szenarien (editor parameter setzen)
- [ ] automatische nummerierung der .bag files und user id


## Instructions:
- Time is measured by starting the bag record when walking on the path

## Parameters
- Ramp of force inside area is defined at robotrainer_bringup/modalities_za_experimental.yaml
	- trapezoid_max_at_percent_radius: 0.75
	- Meaning: At 75 percent of the radius FROM THE CENTER, the max force is applied. So higher value is steeper ramp.

## Topics to record:
- /base/fts_adaptive_force_controller/debug/velocity_output
- /base/fts_adaptive_force_controller/debug/force_input_raw
- /base/fts_adaptive_force_controller/debug/force_input_scaled_limited
- /base/output_data
- /base/virtual_forces/modalities_debug/position
- /base/virtual_forces/modalities_debug/velocity_in
- /base/virtual_forces/modalities_debug/velocity_out
- /base/virtual_forces/modalities_debug/resulting_velocity
- /base/virtual_forces/modalities_debug/resulting_force
   - Diese Kraft ist skaliert auf max_force (default 100N), das heißt alles x100 ergibt die aktuell wirkende Kraft in Newton
- /base/virtual_forces/modalities_debug/status
- /base_laser_back/scan
- /biosensors/polar_oh1/hr
- /biosensors/polar_oh1/ppg_ch0
- /biosensors/polar_oh1/ppg_ch1
- /biosensors/polar_oh1/ppg_ch2
- /biosensors/polar_oh1/ppg_ch3
- /biosensors/polar_oh1/ppi
- /biosensors/polar_oh1/hrv
- /lower_legs_camera/depth_registered/points
- /lower_legs_camera/depth_registered/image_raw
- /lower_legs_camera/depth_registered/camera_info
- /upper_body_camera_realsense/color/camera_info
- /upper_body_camera_realsense/color/image_raw/theora
- /upper_body_camera_realsense/depth/camera_info
- /upper_body_camera_realsense/depth/metadata
- /upper_body_camera_realsense/depth/color/points
- /map
- /map_metadata
- /mobile_robot_pose
- /robotrainer_deviation/current_path_index
- /robotrainer_deviation/robotrainer_deviation
- /robotrainer_deviation/robotrainer_deviation_markers
- /rt2_sca_sync_node/sync_signal
- /toe_detection/toe_positions
- /tf
- /tf_static

# Record only raw data
rosbag record /base/fts_adaptive_force_controller/debug/velocity_output /mobile_robot_pose /base/output_data /lower_legs_camera/depth_registered/points /base_laser_back/scan /map /tf /tf_static

# Record all data
```bash
cd workspace/ros_ws_melodic_robotrainer/src/za_experimental/data/

rosbag record /base/virtual_forces/modalities_debug/position /base/virtual_forces/modalities_debug/velocity_in /base/virtual_forces/modalities_debug/velocity_out /base/virtual_forces/modalities_debug/resulting_velocity /base/virtual_forces/modalities_debug/resulting_force /base/virtual_forces/modalities_debug/status /biosensors/polar_oh1/hr /biosensors/polar_oh1/ppg_ch0 /biosensors/polar_oh1/ppg_ch1 /biosensors/polar_oh1/ppg_ch2 /biosensors/polar_oh1/ppg_ch3 /biosensors/polar_oh1/ppi /biosensors/polar_oh1/hrv /base/fts_adaptive_force_controller/debug/velocity_output /base/fts_adaptive_force_controller/debug/force_input_raw /base/fts_adaptive_force_controller/debug/force_input_scaled_limited /mobile_robot_pose /base/output_data /upper_body_camera_realsense/color/camera_info /upper_body_camera_realsense/color/image_raw/theora /upper_body_camera_realsense/depth/camera_info /lower_legs_camera/depth_registered/points /lower_legs_camera/depth_registered/image_raw /lower_legs_camera/depth_registered/camera_info /upper_body_camera_realsense/depth/metadata /upper_body_camera_realsense/depth/color/points /base_laser_back/scan /map /map_metadata /robotrainer_deviation/current_path_index /robotrainer_deviation/robotrainer_deviation /robotrainer_deviation/robotrainer_deviation_markers /toe_detection/toe_positions /rt2_sca_sync_node/sync_signal /tf /tf_static -o NAME
```

# copy data with scp and ssh with laptop
```bash
scp robotrainer_iras:/home/robotrainer/workspace/ros_ws_melodic_robotrainer/src/za_experimental/data/2025-X.bag /home/andreas/code/robotrainer/bags/
```


## Launch Max Force Test
```bash
srt 
roslaunch za_experimental rt2.launch
# 1. Funknotaus anschalten
# 2. Am RoboTrainer quitieren

rt2_init

rt2_recover

roslaunch robotrainer_panel robotrainer.launch
# 1. Check if localization is correct
# 2. Load scenario

roslaunch za_experimental rqt_reconfigure.launch
# 1. In "FTSAdaptiveForceController" and Tab "Base_Force_Parameterization"
# 2. click: activate_force_parameterization
# 3. follow instructions in console
	# 1. forward
	# 2. left
	# 3. right
	# 4. (turn left)
	# 5. (turn right)
# (4. Für einen neuen Testdurchgang nochmal "activate_force_parameterization" klicken)
# 5. untick "parameterization_activated" to NOT use max force values for the next test
```

## Launch User Study
```bash
# 1. Move RoboTrainer at the beginning of the path out of any force areas
# 2. In robotrainer_panel load and activate scenario
# 3. In rqt_reconfigure activate modalities
    # 1. In  "FTSBaseController/RobotrainerControlActions"
    # 2. switch on tab "Spatial_control_actions"
    # 3. select "spatial_control_action_type: modalities (1)"
    # 4. tick apply_control_actions (Nur EINMAL nach controller starten notwendig)

roslaunch robotrainer_deviation robotrainer_deviation.launch
# After scenario is loaded deviation has to be configured
rosservice call robotrainer_deviation/configure

# Lower Camera
roslaunch gait_parameters_estimation camera.launch

# Extract mobile robot base
roslaunch gait_parameters_estimation extract_mobile_robot_base.launch

# Toe detection
roslaunch gait_parameters_estimation toe_detection.launch

# Pulse Armband
cd ~/workspace/docker/robotrainer_docker_humble
./autostart.sh
# Check if correct bluetooth adapter is used
bluetoothctl info A0:9E:1A:E0:BC:97
# If the output is NOT: "Controller BC:FC:E7:21:3B:E8 iar-ipr-sr3 #2 [default]"
# Ein und AUsstecken von ASUS USB Bluetooth Adapter

# ROS1 Bridge
cd ~/workspace/docker/robotrainer_docker_ros1_bridge
./autostart.sh
```