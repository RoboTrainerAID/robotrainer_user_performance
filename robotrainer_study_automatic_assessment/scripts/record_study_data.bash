bash `rospack find robotrainer_config`/scripts/record_bag_in_background.bash .RoboTrainer/study_record_base.status `rospack find robotrainer_config`/scripts/record_base_values.bash ~/RT2_Data/RoSy_Eval/

#####################################

output_file_prefix="KATE_Automatic_Assessment"

folder="$HOME/Bag_Data"

bag_record_params=""

full_list=()

# Standard
standard=(\ 
/tf /tf_static /map /map_metadata \
)
full_list+=(${standard[*]})
# Laser Scanners
laser_scanner=(\
# /scan_unified \
/base_laser_back/scan \
)
full_list+=(${laser_scanner[*]})
# Base Controller
base_controller=(\
/base/output_data \
/base/fts_adaptive_force_controller/debug/force_input_raw \
/base/fts_adaptive_force_controller/debug/force_input_scaled_limited \
/base/fts_adaptive_force_controller/debug/velocity_output \
)
full_list+=(${base_controller[*]})
# Adaptive Controller
# adaptive_controller=(\
# /base/fts_adaptive_force_controller/adapted_limited_input_force \
# /base/fts_adaptive_force_controller/adaption/maxFT \
# /base/fts_adaptive_force_controller/adaption/scaled_value \
# )
# full_list+=(${adaptive_controller[*]})
# Parameterization
# parametrization=(\
# /base/fts_adaptive_force_controller/parametrization/base/averageDist \
# /base/fts_adaptive_force_controller/parametrization/base/currentDistanceFromStart \
# /base/fts_adaptive_force_controller/parametrization/base/currentRawForce \
# /base/fts_adaptive_force_controller/parametrization/base/resultingForce \
# /base/fts_adaptive_force_controller/parametrization/base/virtualSpringForce \
# /base/fts_adaptive_force_controller/parametrization/max_force_result \
# )
# full_list+=(${parametrization[*]})
# Modalities
modalities=(\
/base/virtual_forces/modalities_debug/position \
/base/virtual_forces/modalities_debug/velocity_in \
/base/virtual_forces/modalities_debug/velocity_out \
/base/virtual_forces/modalities_debug/resulting_velocity \
/base/virtual_forces/modalities_debug/resulting_force \
/base/virtual_forces/modalities_debug/status \
)
full_list+=(${modalities[*]})
# Robots state
robot_state=(\
/mobile_robot_pose \
)
full_list+=(${robot_state[*]})
# Persons
persons=(\
/toe_detection/toe_positions \
/biosensors/polar_oh1/hr \
/biosensors/polar_oh1/ppg_ch0 \
/biosensors/polar_oh1/ppg_ch1 \
/biosensors/polar_oh1/ppg_ch2 \
/biosensors/polar_oh1/ppg_ch3 \
/biosensors/polar_oh1/ppi \
/biosensors/polar_oh1/hrv \
)
full_list+=(${persons[*]})
# Upper Body Camera
upper_body_camera=(\
/upper_body_camera_realsense/color/camera_info \
/upper_body_camera_realsense/color/image_raw/theora \
/upper_body_camera_realsense/depth/camera_info \
/upper_body_camera_realsense/depth/metadata \
/upper_body_camera_realsense/depth/color/points \
)
full_list+=(${upper_body_camera[*]})
# Lower Legs Camera
lower_body_camera=(\
/lower_legs_camera/depth_registered/camera_info \
/lower_legs_camera/depth_registered/image_raw \
/lower_legs_camera/depth_registered/points \
)
full_list+=(${lower_body_camera[*]})
# Study
study=(\
/rt2_sca_sync_node/sync_signal \
/robotrainer_deviation/current_path_index \
/robotrainer_deviation/robotrainer_deviation \
/robotrainer_deviation/robotrainer_deviation_markers \
)
full_list+=(${study[*]})

##########################################


#!/bin/bash
trap "kill 0" EXIT

# File to get commands and store pid
file="$HOME/$1"
if [ -z "$1" ]; then
  echo "No file provided exiting."
  exit
fi

# Configuration file to source
config=$2
if [ -z "$2" ]; then
  echo "No configuration file to source provided exiting."
  exit
fi

# Folder for the Bag
folder=$3

# Bag file prefix
output_file_prefix=$4

source $config

pid_file="$file.pid"

echo "" > $file
echo "" > $pid_file

started=false
parameter=""
pid=-1

while :
do 
    first_line=$(head -n 1 $file)  
    if [ "$first_line" = "START" ]  && [ "$started" = false ]; then
        second_line=$(head -n 2 $file | tail -1)
        echo "" > $file
        if [ ! -z "$second_line" ]; then
            output_file_prefix=$second_line
            echo "Using prefix: $output_file_prefix"
        fi
        rosbag record $bag_record_params -p -o $output_file_prefix ${full_list[*]} 
        echo "The rosbag has finished"
    fi
    # FIXME: If 'fg' than the process has to be stopped manually
    if [ "$started" = true ]; then
#         kill -0 $pid
#         status=$?
        if [ "$status" = "1" ]; then
            started=false
            echo "Process with PID $pid has stopped!"
            pid="-----"
            echo "-" > $file
            echo $pid > $pid_file
        fi
    fi
    sleep 0.1
done


#####################################

