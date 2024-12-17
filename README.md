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

ign gazebo ros_ws/src/src/guided_navigation/world/camera_world.sdf

ros2 launch guided_navigation guided_simulation.launch.py

## Configuracao para deteccao com o modelo de treinamento da yolo:

export ROBOFLOW_API_KEY="aFkoLbgUAThELZEBkgQ5"


## Service para visualizar a simulacao

ros2 service call /set_camera_active guided_navigation/srv/SetCameraActive "{camera_id: 7, activate: true}"

## Status da simulacao:

ros2 run guided_navigation simulation_monitor.py

## Planner:

ros2 run guided_navigation planner.py


## Rodar triangulacao:

source install/local_setup.bash

ros2 run guided_navigation triangulation.py

## Rodar Filtro de particulas
source install/local_setup.bash

ros2 run guided_navigation filtro_particulas.py

## Comparar posicao estimada com odometria (iniciar o robo em 0 0 0 0 0 0)
source install/local_setup.bash

ros2 run guided_navigation compare_position.py

## RViz 
rviz2 -d ros_ws/src/src/guided_navigation/rviz/filtro.rviz


## camera stereo
sudo apt-get install ros-humble-camera-calibration

ros2 launch guided_navigation stereo-guided_simulation.launch.py

### Calibracao 
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.024 --approximate 0.1 --ros-args --remap left:=/left/image_raw --remap left_camera:=/left/camera_info --remap right:=/right/image_raw --remap right_camera:=/right/camera_info

##ajuda na simulacao

ign topic -e -t /world/empty/pose/info | grep -A 10 'name: "rgbd_camera_0"'



