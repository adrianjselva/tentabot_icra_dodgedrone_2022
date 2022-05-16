#!/bin/bash

# Author: Neset Unver Akmandor (NUA)
#         Gary M. Lvov (GML)     
#         Hongyu Li (LHY) 
# E-Mail: akmandor.n@northeastern.edu
#         lvov.g@nor4theastern.edu
#         li.hongyu1@northeastern.edu

# GML TODO: verify new auto install

sudo apt update
sudo apt install python3-pip

tentabot_path=$(pwd)
cd ../../
catkin_path=$(pwd)
cd "${tentabot_path}" || exit

# 3.1 Install flexible-collision-library/fcl following their instructions using CMake.
#------ Install libcdd sub-dep
mkdir "dependencies"
cd "dependencies" || exit

git clone https://github.com/danfis/libccd.git
cd libccd || exit
mkdir build
cd build || exit
cmake -G "Unix Makefiles" -DBUILD_SHARED_LIBS=ON ..
make
make install
cd ../../

#------ Install eigen sub-dep
apt-get install libeigen3-dev

#------ Install fcl
git clone https://github.com/flexible-collision-library/fcl.git
cd "fcl" || exit
mkdir build
cd "build" || exit
cmake ..
cd ../../

cd "${catkin_path}/src" || exit

# 3.3 Install rotors_simulator package into the src folder.
git clone https://github.com/ethz-asl/rotors_simulator.git

# 3.4 Install 'noetic-akmandor' branch of turtlebot3 package into the src folder.
git clone https://github.com/RIVeR-Lab/turtlebot3.git  #'noetic-akmandor' branch*
cd "turtlebot3" || exit
git checkout noetic-akmandor
cd ..

# 3.5 Install 'noetic-akmandor' branch of LMS1xx package into the src folder.
git clone https://github.com/RIVeR-Lab/LMS1xx.git  #'noetic-akmandor' branch*
cd "LMS1xx" || exit
git checkout noetic-akmandor
cd ..

# 3.6 Install 'noetic-akmandor' branch of geometry2 package into the src folder.
git clone https://github.com/RIVeR-Lab/geometry2.git  #'noetic-akmandor' branch*
cd "geometry2" || exit
git checkout noetic-akmandor
cd ..

# 3.7 Install catkin-simple package into the src folder.
git clone https://github.com/catkin/catkin_simple.git

# 3.8 Install forest_gen package into the src folder.
git clone https://github.com/ethz-asl/forest_gen.git

# 3.9 Install mav_comm package into the src folder.
git clone https://github.com/ethz-asl/mav_comm.git

# 3.10 Install octomap_rviz_plugins package into the src folder.
git clone https://github.com/OctoMap/octomap_rviz_plugins.git

# 3.11 Install pedsim_ros package into the src folder.
git clone https://github.com/srl-freiburg/pedsim_ros.git

# 3.12 Install openai-ros package into the src folder.
git clone https://github.com/RIVeR-Lab/openai_ros.git

# 3.13 Install tentabot package into the src folder.
# git clone https://github.com/RIVeR-Lab/tentabot.git

# Install stretch_ros package into src folder
git clone https://github.com/hello-robot/stretch_ros.git

# Install realsense package into src folder
git clone https://github.com/pal-robotics/realsense_gazebo_plugin.git

# Install jackal package into src folder
git clone https://github.com/jackal/jackal.git


# Install husky package into src folder
git clone https://github.com/husky/husky.git

# Install joystick drivers into ssrc folder
git clone https://github.com/ros-drivers/joystick_drivers.git
cd "joystick_drivers" ||exit
git checkout kinetic-devel
cd ..

# install navrep 
git clone https://github.com/ethz-asl/navrep.git # need to run setup.py install


# 3.15 Install ROS dependencies
sudo apt-get install libsuitesparse-dev
sudo apt-get install libnlopt-dev

# 3.16 Install other ROS dependencies using rosdep tool:
cd ..
wait
source /opt/ros/noetic/setup.bash
rosdep install -i --from-path src --rosdistro noetic -y

# 3.17 Install Python dependencies
pip install stable-baselines3[extra] #only neeeded for tentabot drl
pip install GitPython
pip install squaternion
