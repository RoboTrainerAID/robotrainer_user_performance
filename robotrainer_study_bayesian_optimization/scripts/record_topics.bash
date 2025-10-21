#!/bin/bash

TOPICS_YAML=$(rospack find robotrainer_study_bayesian_optimization)/config/topics_to_record.yaml
CONFIG_YAML=$(rospack find robotrainer_study_bayesian_optimization)/config/user_study_manager.yaml
STATUS_FILE=$(rospack find robotrainer_study_bayesian_optimization)/status/user_study_manager.status

if [ ! -f "$STATUS_FILE" ]; then
  echo "Status file not found: $STATUS_FILE"
  exit 1
fi

# Safely read the status file using a shared lock on the data file itself.
# The `flock` command will wait until it can acquire the lock, then execute `readarray`.
STATUS=() # Initialize as an empty array
flock -s "$STATUS_FILE" -c "readarray -t STATUS < $STATUS_FILE"

# Check if readarray was successful
if [ ${#STATUS[@]} -lt 4 ]; then
  echo "Error: Failed to read status file or file is incomplete."
  exit 1
fi

STUDY_NAME="${STATUS[0]}"
USER_ID="${STATUS[1]}"
TASK_ID="${STATUS[2]}"
TRIAL="${STATUS[3]}"

# Format bag name (same as user_study_manager.py)
BAG_NAME="${STUDY_NAME}_U${USER_ID}_${TASK_ID}_${TRIAL}"

# Extract list of topics from the simplified YAML
TOPICS=$(python3 -c "
import yaml
with open('$TOPICS_YAML') as f:
    data = yaml.safe_load(f)
topics = data.get('topics_to_record_and_check_for_messages', [])
if isinstance(topics, list):
    print(' '.join(str(t) for t in topics if isinstance(t, str)))
")

if [ -z "$TOPICS" ]; then
  echo "No valid topics found in $TOPICS_YAML"
  exit 1
fi

BAG_FOLDER=$(python3 -c "
import yaml
with open('$CONFIG_YAML') as f:
    data = yaml.safe_load(f)
print(data.get('bag_folder_path', ''))
")

BAG_PATH="${BAG_FOLDER}/${BAG_NAME}"
rosbag record -p -o "$BAG_PATH" $TOPICS
