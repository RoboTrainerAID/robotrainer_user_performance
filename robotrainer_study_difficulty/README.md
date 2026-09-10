# User study Difficulty

## Instructions:
- Time is measured by starting the bag record when walking on the path

## Parameters
- Ramp of force inside area is defined at robotrainer_bringup/modalities_za_experimental.yaml
	- trapezoid_max_at_percent_radius: 0.75
	- Meaning: At 75 percent of the radius FROM THE CENTER, the max force is applied. So higher value is steeper ramp.

## Launch Difficulty study
```bash
srt 
roslaunch za_experimental rt2.launch
# 1. Funknotaus anschalten
# 2. Am RoboTrainer quitieren

rt2_init

# Nur notwendig nach Sicherheitsstopp
rt2_recover

roslaunch robotrainer_panel robotrainer.launch
# 1. Check if localization is correct

roslaunch robotrainer_study_difficulty user_study_nodes.launch

rt2_start_bo
# Wait for BO to successfully start

rt2_start_bridge

rt2_start_polar
# rostopic echo /biosensors/polar_oh1/hr

roslaunch robotrainer_study_difficulty user_study_manager.launch

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
cd /media/robotrainer/RoSylerNT_Eval/KATE_DIFFICULTY/
mv -v ~/workspace/docker/robotrainer_docker_bayesian_optimization/data/* .

# copy data from external hard drive to NAS
rsync -av --ignore-existing ./KATE_DIFFICULTY workstation:/iras/users/zaan0001/robotrainer/
```

## Create gait data after recording bag files
```bash
# Plug in external hard drive into tux laptop
# make folder writable from other user
sudo chmod a+w -R /media/andreas/RoSylerNT_Eval/KATE_DIFFICULTY/bags/

# start melodic docker container and mount external hard drive
./code/docker/robotrainer_docker_melodic/start_docker.sh
    -v /media/andreas/RoSylerNT_Eval/KATE_DIFFICULTY:/home/docker/ros_ws/robotrainer/KATE_DIFFICULTY \

# Run script to iterate folder in container
# Change folder path for input and output
INPUT_FOLDER="/home/docker/ros_ws/robotrainer/KATE_DIFFICULTY/bags/U005_BO_wrench_force_y_max_qUCB/raw"
roscore &
./src/gait_parameters_estimation/gait_parameters_estimation/iterate_folder_and_estimate_gait.bash
```

## Visualize data
```bash
roslaunch robotrainer_study_difficulty rviz_scenario_and_data.launch

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
