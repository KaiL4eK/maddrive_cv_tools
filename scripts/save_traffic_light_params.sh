#!/bin/bash
FILEPATH="$HOME/.maddrive_config/traffic_light_config.yaml"
NAMESPACE="md_config/traffic_light"

mkdir -p $(dirname $FILEPATH)
echo "Saving to $FILEPATH"
rosparam dump $FILEPATH $NAMESPACE
