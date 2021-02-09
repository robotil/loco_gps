# Copyright (C) 2020 Ghost Robotics - All Rights Reserved
# Unauthorized copying of this file, via any medium is strictly prohibited
# Proprietary and confidential

#!/bin/bash

# This script is run on system startup using /etc/systemd/system/ghost.service

# Source
source /home/ghost/.environment_vars.bash
source /home/ghost/loco_ws/devel/setup.bash

roslaunch gps_goal gps_goal.launch
echo "Started Loco"

