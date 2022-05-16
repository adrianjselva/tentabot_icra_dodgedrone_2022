#!/bin/bash

source /opt/ros/noetic/setup.bash
cd "${1}" || exit
source "../../devel/setup.bash" || exit

cd "${2}" || exit;
cd "../envtest/ros/" || exit
python3 "evaluation_node.py"

rostopic pub "/kingfisher/dodgeros_pilot/off" std_msgs/Empty "{}" -1
rostopic pub "/kingfisher/dodgeros_pilot/reset_sim" std_msgs/Empty "{}" -1