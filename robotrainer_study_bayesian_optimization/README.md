# User study automatic assessment

## Instructions:
- Time is measured by starting the bag record when walking on the path

## Parameters
- Ramp of force inside area is defined at robotrainer_bringup/modalities_za_experimental.yaml
	- trapezoid_max_at_percent_radius: 0.75
	- Meaning: At 75 percent of the radius FROM THE CENTER, the max force is applied. So higher value is steeper ramp.

## Launch Bayesian Optimization
```bash
srt 
roslaunch za_experimental rt2.launch
# 1. Funknotaus anschalten
# 2. Am RoboTrainer quitieren

rt2_init

# Nur wenn notwendig nach sicherheitsstopp
rt2_recover

roslaunch robotrainer_panel robotrainer.launch
# 1. Check if localization is correct

roslaunch robotrainer_study_bayesian_optimization user_study_nodes.launch

rt2_start_bo
# Wait for BO to successfully start

rt2_start_polar
# ./scripts/start_polar_oh1.bash
# rostopic echo /biosensors/polar_oh1/hr

roslaunch robotrainer_study_bayesian_optimization user_study_manager.launch

roslaunch za_experimental rqt_reconfigure.launch
# 1. In rqt_reconfigure activate modalities
    # 1. In  "FTSBaseController/RobotrainerControlActions"
    # 2. switch on tab "Spatial_control_actions"
    # 3. select "spatial_control_action_type: modalities (1)"
    # 4. tick apply_control_actions 
# (Nur EINMAL nach controller starten notwendig)

# First task Experiment:
# 1. In rqt_reconfigure in left list, click refresh
# 2. Click on robotrainer_user_study_manager
# 3. Click next_user (load new scenario and trigger topic check)
# 4. Start rosbag record

# For each following Task:
# 1. Drive RoboTrainer back to its starting position
# 2. In rqt_reconfigure click next_task (triggers /update service)
# 3. Wait for all messages beeing green in user_study_manager
# 4. Start rosbag record


rt2_start_bag
# ./scripts/record_topics.bash
# DATA FOLDER: ~/workspace/docker/robotrainer_docker_bayesian_optimization/data/bags
```

## Transfer data to external hard drive
```bash
# 1. Plug external hard drive into usb3.0 port
udisksctl mount -b /dev/sda1
# -> student-admin -> PW
cd /media/robotrainer/RoSylerNT_Eval/KATE_BO/
mv -v ~/workspace/docker/robotrainer_docker_bayesian_optimization/data/* .


# copy data with scp and ssh with laptop 
scp robotrainer_iras:/home/robotrainer/workspace/ros_ws_melodic_robotrainer/src/robotrainer_user_performance/robotrainer_study_bayesian_optimization/data/2025-X.bag /home/andreas/code/robotrainer/bags/

# copy data from external hard drive to NAS
rsync -av --ignore-existing ./KATE_AA workstation:/home/zaan0001/nas/robotrainer/
```

## Visualize data
```bash
roslaunch robotrainer_study_bayesian_optimization rviz_scenario_and_data.launch

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
