# Copyright (C) 2020 Ghost Robotics - All Rights Reserved
# Unauthorized copying of this file, via any medium is strictly prohibited
# Proprietary and confidential

#!/bin/bash

# This script is run on system startup using /etc/systemd/system/ghost.service

# Source
#source /home/ghost/.environment_vars.bash
echo "Here"
source /home/ghost/loco_ws/devel/setup.bash
echo "HereHere"
sleep 60

echo "Now"
#rosnode list &
roslaunch loco_gps loco.launch  2>&1> /home/ghost/loco.log &

echo "Started Loco"

