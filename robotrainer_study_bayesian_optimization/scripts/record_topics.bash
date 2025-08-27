#!/bin/bash

CONFIG_YAML=$(rospack find robotrainer_study_bayesian_optimization)/config/topics_to_record.yaml
STATUS_FILE=$(rospack find robotrainer_study_bayesian_optimization)/status/user_study_manager.status
BAG_FOLDER=$(rospack find robotrainer_study_bayesian_optimization)/data

if [ ! -f "$STATUS_FILE" ]; then
  echo "Status file not found: $STATUS_FILE"
  exit 1
fi

readarray -t STATUS < "$STATUS_FILE"
STUDY_NAME="${STATUS[0]}"
USER_ID="${STATUS[1]}"
TASK_ID="${STATUS[2]}"
TRIAL="${STATUS[3]}"

# Format bag name (same as user_study_manager.py)
BAG_NAME="${STUDY_NAME}_U${USER_ID}_${TASK_ID}-${TRIAL}"

# Extract list of topics from the simplified YAML
TOPICS=$(python3 -c "
import yaml
with open('$CONFIG_YAML') as f:
    data = yaml.safe_load(f)
topics = data.get('topics_to_record_and_check_for_messages', [])
if isinstance(topics, list):
    print(' '.join(str(t) for t in topics if isinstance(t, str)))
")

if [ -z "$TOPICS" ]; then
  echo "No valid topics found in $CONFIG_YAML"
  exit 1
fi

# echo "Bag file name: $BAG_NAME"
BAG_PATH="${BAG_FOLDER}/${BAG_NAME}"
rosbag record -p -o "$BAG_PATH" $TOPICS
