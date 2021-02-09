#!/bin/bash

source /home/ghost/loco_ws/devel/setup.bash
#rosnode kill /loco_gps_node  /gps_goal /rosbridge_websocket

if [ "$(ps -ef | grep -i loco_ | grep -v grep | grep -v locokill.sh | grep -v chrome | awk '{print $2}')" ]; then # or $(rosnode list)
    for i in $(ps -ef | grep -i loco_ | grep -v grep | grep -v locokill.sh | grep -v chrome | awk '{print $2}')
    do
        kill -9 $i;
        echo "Killing " $i "...killed?";
    done
fi

