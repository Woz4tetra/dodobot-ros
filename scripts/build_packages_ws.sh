#!/usr/bin/env bash

BASE_DIR=$(realpath "$(dirname $0)")

DEPENDENCIES_WS=$HOME/packages_ros_ws
DEPENDENCIES_WS_SRC=${DEPENDENCIES_WS}/src

sudo apt-get install -y portaudio19-dev python3-pyaudio
sudo -H pip3 install -r requirements.txt

mkdir -p ${DEPENDENCIES_WS_SRC}
cd ${DEPENDENCIES_WS_SRC}

packages=(
    https://github.com/wjwwood/serial
    https://github.com/ros-drivers/joystick_drivers.git
    https://github.com/ros/geometry2.git
    https://github.com/ros-planning/navigation.git
    https://github.com/rst-tu-dortmund/teb_local_planner
    https://github.com/introlab/rtabmap_ros.git
    https://github.com/IntelRealSense/realsense-ros.git
    https://github.com/ros-perception/vision_opencv.git
    https://github.com/ros-perception/vision_msgs.git
    https://github.com/ros-planning/navigation_msgs.git
    https://github.com/magazino/move_base_flex.git
    https://github.com/introlab/find-object.git
    https://github.com/pal-robotics/ddynamic_reconfigure.git
    https://github.com/ros-perception/perception_pcl.git
    https://github.com/OctoMap/octomap_msgs.git
    https://github.com/ros-perception/pcl_msgs.git
    https://github.com/AprilRobotics/apriltag_ros.git
    https://github.com/rst-tu-dortmund/costmap_converter.git
)

branches=(
    master
    main
    noetic-devel
    noetic-devel
    noetic-devel
    noetic-devel
    development
    noetic
    noetic-devel
    ros1
    noetic-devel
    noetic-devel
    kinetic-devel
    melodic-devel
    melodic-devel
    noetic-devel
    master
    master
)

len=${#packages[@]}
for (( i=0; i<$len; i++ )); do
    git clone ${packages[i]} --branch ${branches[i]}
done

cd ..

rosdep install --from-paths src --ignore-src --rosdistro=noetic -y -r

find ${DEPENDENCIES_WS_SRC} -type f -name CMakeLists.txt -exec sed -i'' -e 's/Boost REQUIRED python37/Boost REQUIRED python/g' {} +

find ${DEPENDENCIES_WS_SRC} -type f -name CMakeLists.txt -exec sed -i'' -e 's/find_package(realsense2 2.45.0)/find_package(realsense2 2.41.0)/g' {} +

sed -i -e 's/${G2O_INCREMENTAL_LIB}/#${G2O_INCREMENTAL_LIB}/g'  ${DEPENDENCIES_WS_SRC}/teb_local_planner/cmake_modules/FindG2O.cmake

catkin_make -j2
success=$?
if [[ $success -eq 0 ]];
then
    echo "Packages built successfully"
else
    echo "Something went wrong!"
    exit 1
fi
