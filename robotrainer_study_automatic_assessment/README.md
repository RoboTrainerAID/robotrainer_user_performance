# User study automatic assessment

## Instructions:
- Time is measured by starting the bag record when walking on the path

## Parameters
- Ramp of force inside area is defined at robotrainer_bringup/modalities_za_experimental.yaml
	- trapezoid_max_at_percent_radius: 0.75
	- Meaning: At 75 percent of the radius FROM THE CENTER, the max force is applied. So higher value is steeper ramp.

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

roslaunch robotrainer_study_automatic_assessment user_study_nodes.launch

rt2_start_polar
# ./scripts/start_polar_oh1.bash

roslaunch robotrainer_study_automatic_assessment user_study_manager.launch
# 1. In rqt_reconfigure in left list, click refresh
# 2. Click on robotrainer_user_study_manager
# 3. Click next_user (load new scenario and trigger topic check)
# 4. In rqt_reconfigure activate modalities
    # 1. In  "FTSBaseController/RobotrainerControlActions"
    # 2. switch on tab "Spatial_control_actions"
    # 3. select "spatial_control_action_type: modalities (1)"
    # 4. tick apply_control_actions (Nur EINMAL nach controller starten notwendig)


# For each Task:
# 1. Drive RoboTrainer back to its starting position
# 2. In rqt_reconfigure click next_task
# 3. Wait for all messages beeing green in user_study_manager
# 4. Start rosbag record

rostopic echo /biosensors/polar_oh1/hr

rt2_start_bag
# ./scripts/record_topics.bash
# DATA FOLDER: ./robotrainer_user_performance/robotrainer_study_automatic_assessment/data/
```

## Transfer data to external hard drive
```bash
# 1. Plug external hard drive into usb3.0 port
udisksctl mount -b /dev/sda1
# -> student-admin -> PW
cd /media/robotrainer/RoSylerNT_Eval/KATE_AA/
mv -v ~/workspace/ros_ws_melodic_robotrainer/src/robotrainer_user_performance/robotrainer_study_automatic_assessment/data/*.bag .


# copy data with scp and ssh with laptop 
scp robotrainer_iras:/home/robotrainer/workspace/ros_ws_melodic_robotrainer/src/robotrainer_user_performance/robotrainer_study_automatic_assessment/data/2025-X.bag /home/andreas/code/robotrainer/bags/

# copy data from external hard drive to NAS
rsync -av --ignore-existing ./KATE_AA workstation:/home/zaan0001/nas/robotrainer/
```

## Visualize data
```bash
roslaunch robotrainer_study_automatic_assessment rviz_scenario_and_data.launch

rosbag play /path/to/your.bag
```

## Todos

- [x] Soll alle topics kontrollieren ob gepublished wird
- [x] Richtige Bluetooth adapter id kontrollieren
- [x] Feedback geben welches topic fehlt oder freigabe für test geben
- [x] Kontrollieren ob alle nodes laufen?
- [x] launch files für alle nodes
- [x] automatisch welchsel zwischen verschiedenen Szenarien (editor parameter setzen)
- [x] automatische nummerierung der .bag files und user id
- [x] alle task ids eintragen
- [x] force area topics ausnahmen
- [x] y-balance device zusammenbauen
- [x] Klebebänder auf boden entfernen
- [x] Sicherheitseinweisung Akkus
- [x] Fragebogen überprüfen
