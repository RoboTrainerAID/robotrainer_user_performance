#!/bin/bash

if [ -z "$folder" ]; then
  echo "No data folder provided. Using default Folder '$HOME/Bag_Data'."
  folder="$HOME/Bag_Data"
else
  echo "Using '$folder' folder to store data"
fi

if [ -z "$output_file_prefix" ]; then
  echo "No output file prefix provided. Using default 'RoSylerNT_User_Sensors'."
  output_file_prefix="RoSylerNT_User_Sensors"
fi

if [ ! -d "$folder" ]; then
  echo "Creating data folder $folder"
  mkdir -p $folder
fi

cd $folder

bag_record_params="--udp --duration=30"

full_list=()

# Standard
standard=(\
/tf /tf_static /map /map_metadata  /amcl_pose \
/base/joint_states \
/base/odometry_controller/odometry \
/scan_unified \
/joint_states \
/robotrainer/mobile_robot_pose \
)
full_list+=(${standard[*]})
# Laser Scanners
laser_scanner=(\
/scan_unified \
)
full_list+=(${laser_scanner[*]})
# Upper Body Camera
upper_body_camera=(\
/upper_body_camera/depth_registered/camera_info \
/upper_body_camera/depth_registered/image_raw \
/upper_body_camera/depth_registered/points \
/upper_body_camera/rgb/camera_info \
/upper_body_camera/rgb/image_raw \
)
full_list+=(${upper_body_camera[*]})
# Lower Legs Camera
lower_body_camera=(\
/lower_legs_camera/depth_registered/camera_info \
/lower_legs_camera/depth_registered/image_raw \
/lower_legs_camera/depth_registered/points \
/lower_legs_camera/rgb/camera_info \
/lower_legs_camera/rgb/image_raw \
)
full_list+=(${lower_body_camera[*]})
# Study
study=(\
/rt2_sca_sync_node/sync_signal \
/robotrainer_user_study_manager/study_status \
)
full_list+=(${study[*]})

