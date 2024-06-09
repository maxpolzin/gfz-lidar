#!/bin/bash

ros2 bag record /rslidar_packets /rslidar_points --max-cache-size 1000000000 -s mcap --qos-profile-overrides-path ./qos_override.yaml
