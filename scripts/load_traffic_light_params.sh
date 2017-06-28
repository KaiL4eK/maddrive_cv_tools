#!/bin/bash
FILEPATH="$HOME/.maddrive_config/traffic_light_config.yaml"
NAMESPACE="md_config/traffic_light"

if [ -f $FILEPATH ]; then
    echo "Loading from $FILEPATH"
    rosparam load $FILEPATH $NAMESPACE
else
    echo "No saved params"
fi
