#!/bin/bash
FILEPATH="$HOME/.maddrive_config/cv_traffic_light.yaml"
NAMESPACE="/md_cv_traffic_light"

if [ -f $FILEPATH ]; then
    echo "Loading from $FILEPATH"
    rosparam load $FILEPATH $NAMESPACE
else
    echo "No saved params"
fi
