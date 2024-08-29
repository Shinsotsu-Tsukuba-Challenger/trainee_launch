#!/usr/bin/env bash

path=$(echo $1 | sed 's|/install/.*/share|/src|')
ros2 run nav2_map_server map_saver_cli -f $path