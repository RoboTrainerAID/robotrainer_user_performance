#!/bin/bash

if [ -z "$folder" ]; then
  echo "No data folder provided. Using default Folder '$HOME/Bag_Data'."
  folder="$HOME/Bag_Data"
else
  echo "Using '$folder' folder to store data"
fi

if [ -z "$output_file_prefix" ]; then
  echo "No output file prefix provided. Using default 'RoSylerNT'."
  output_file_prefix="RoSylerNT_User_Sensors"
fi

if [ ! -d "$folder" ]; then
  echo "Creating data folder $folder"
  mkdir -p $folder
fi

cd $folder

bag_record_params=""

full_list=()

# Standard
standard=(\ 
/tf /tf_static /map /map_metadata  /amcl_pose \
/base/joint_states \
/joint_states \
/base/odometry_controller/odometry \
)
full_list+=(${standard[*]})
# Laser Scanners
laser_scanner=(\
/scan_unified \
)
full_list+=(${laser_scanner[*]})
# /scan_unified \
# /joint_states \
# Base Controller
base_controller=(\
/base/fts_adaptive_force_controller/debug/force_input_raw \
/base/fts_adaptive_force_controller/debug/force_input_scaled_limited \
/base/fts_adaptive_force_controller/debug/velocity_output \
/base/fts_adaptive_force_controller/debug/admittance_velocity \
/base/fts_adaptive_force_controller/debug/platform_hw_velocity \
)
full_list+=(${base_controller[*]})
# Adaptive Controller
adaptive_controller=(\
/base/fts_adaptive_force_controller/adapted_limited_input_force \
/base/fts_adaptive_force_controller/adaption/maxFT \
/base/fts_adaptive_force_controller/adaption/scaled_value \
)
full_list+=(${adaptive_controller[*]})
# Parameterization
parametrization=(\
/base/fts_adaptive_force_controller/parametrization/base/averageDist \
/base/fts_adaptive_force_controller/parametrization/base/currentDistanceFromStart \
/base/fts_adaptive_force_controller/parametrization/base/currentRawForce \
/base/fts_adaptive_force_controller/parametrization/base/resultingForce \
/base/fts_adaptive_force_controller/parametrization/base/virtualSpringForce \
/base/fts_adaptive_force_controller/parametrization/max_force_result \
)
full_list+=(${parametrization[*]})
# Modalities
modalities=(\
/base/virtual_forces/modalities_debug/position \
/base/virtual_forces/modalities_debug/velocity_in \
/base/virtual_forces/modalities_debug/velocity_out \
/base/virtual_forces/modalities_debug/resulting_velocity \
/base/virtual_forces/modalities_debug/resulting_force \
)
full_list+=(${modalities[*]})
# Persons
persons=(\
/leg_detection/people_msg_stamped \
)
full_list+=(${persons[*]})
# Robots state
robot_state=(\
/robotrainer/mobile_robot_pose \
/emergency_stop_state \
)
full_list+=(${robot_state[*]})
# Study
study=(\
/rt2_sca_sync_node/sync_signal \
/robotrainer_user_study_manager/study_status \
/robotrainer_deviation/current_path_index \
/robotrainer_deviation/robotrainer_deviation \
/robotrainer_deviation/robotrainer_deviation_markers \
)
full_list+=(${study[*]})
