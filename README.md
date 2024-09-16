# guided_navigation

## Ubuntu Version : 22.02.04 LTS (Jammy)
https://www.releases.ubuntu.com/jammy/

## ROS 2 Humble Hawksbill
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

## Gazebo Fortress
https://gazebosim.org/docs/fortress/install_ubuntu/

### ROS2 and Gazebo Bridge communication: 
sudo apt-get install ros-humble-ros-ign-bridge

https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_bridge

### Colocar isso no bashrc:
export GZ_SIM_RESOURCE_PATH=~/ros2_wc/src/guided_navigation/models

## Rodar simulacao:

colcon build --packages-select guided_navigation

source install/local_setup.bash

ros2 launch guided_navigation guided_simulation.launch.py

## Rodar triangulacao:

source install/local_setup.bash

ros2 run guided_navigation triangulation.py