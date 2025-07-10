#!/bin/bash

cd /home/elf/slam/fast_livo2_ws
source install/setup.bash
source /home/elf/slam/livox/livox_ws/install/setup.bash --extend
ros2 run livox_converter livox_msg_converter