#!/bin/bash
FILEPATH="$HOME/.maddrive_config/cv_traffic_light.yaml"
NAMESPACE="/md_cv_traffic_light"
mkdir -p $(dirname $FILEPATH)
echo "Saving to $FILEPATH"
rosparam dump $FILEPATH $NAMESPACE
