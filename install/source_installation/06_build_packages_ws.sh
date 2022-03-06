#!/usr/bin/env bash

BASE_DIR=$(realpath "$(dirname $0)")
APPLY_PATCHES=${1:-}

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
    git@github.com:frc-88/navigation.git
    https://github.com/rst-tu-dortmund/teb_local_planner
    https://github.com/introlab/rtabmap_ros.git
    https://github.com/IntelRealSense/realsense-ros.git
    https://github.com/ros-perception/vision_opencv.git
    https://github.com/ros-perception/vision_msgs.git
    git@github.com:frc-88/navigation_msgs.git
    https://github.com/magazino/move_base_flex.git
    https://github.com/introlab/find-object.git
    https://github.com/pal-robotics/ddynamic_reconfigure.git
    https://github.com/ros-perception/perception_pcl.git
    https://github.com/OctoMap/octomap_msgs.git
    https://github.com/ros-perception/pcl_msgs.git
    https://github.com/AprilRobotics/apriltag_ros.git
    https://github.com/rst-tu-dortmund/costmap_converter.git
    https://github.com/ros-perception/image_pipeline.git
    https://github.com/ros-perception/image_common.git
    https://github.com/robopeak/rplidar_ros.git
    https://github.com/ros-perception/laser_filters.git
    https://github.com/cra-ros-pkg/robot_localization
    https://github.com/ros-geographic-info/geographic_info.git
    https://github.com/ros-geographic-info/unique_identifier.git
    https://github.com/ros-perception/laser_filters.git
    https://github.com/ros-perception/slam_gmapping.git
    https://github.com/ros-perception/openslam_gmapping.git
    https://github.com/ros-drivers/usb_cam.git
    https://github.com/ros-perception/image_transport_plugins
    https://github.com/ros/rosbag_snapshot.git
    https://github.com/stereolabs/zed-ros-wrapper
    https://github.com/stereolabs/zed-ros-examples
    https://github.com/alireza-hosseini/ipcamera_driver
    https://github.com/ros-teleop/twist_mux.git
    https://github.com/ros-teleop/twist_mux_msgs.git
    https://github.com/iralabdisco/ira_laser_tools.git
    https://github.com/ros-perception/pointcloud_to_laserscan.git
)

branches=(
    main      # https://github.com/wjwwood/serial
    main    # https://github.com/ros-drivers/joystick_drivers.git
    noetic-devel    # https://github.com/ros/geometry2.git
    noetic-devel    # git@github.com:frc-88/navigation.git
    noetic-devel    # https://github.com/rst-tu-dortmund/teb_local_planner
    noetic-devel    # https://github.com/introlab/rtabmap_ros.git
    development     # https://github.com/IntelRealSense/realsense-ros.git
    noetic      # https://github.com/ros-perception/vision_opencv.git
    noetic-devel    # https://github.com/ros-perception/vision_msgs.git
    ros1    # git@github.com:frc-88/navigation_msgs.git
    noetic-devel    # https://github.com/magazino/move_base_flex.git
    noetic-devel    # https://github.com/introlab/find-object.git
    kinetic-devel   # https://github.com/pal-robotics/ddynamic_reconfigure.git
    melodic-devel   # https://github.com/ros-perception/perception_pcl.git
    melodic-devel   # https://github.com/OctoMap/octomap_msgs.git
    noetic-devel    # https://github.com/ros-perception/pcl_msgs.git
    master      # https://github.com/AprilRobotics/apriltag_ros.git
    master      # https://github.com/rst-tu-dortmund/costmap_converter.git
    noetic      # https://github.com/ros-perception/image_pipeline.git
    noetic-devel    # https://github.com/ros-perception/image_common.git
    master      # https://github.com/robopeak/rplidar_ros.git
    kinetic-devel   # https://github.com/ros-perception/laser_filters.git
    noetic-devel    # https://github.com/cra-ros-pkg/robot_localization
    master      # https://github.com/ros-geographic-info/geographic_info.git
    master      # https://github.com/ros-geographic-info/unique_identifier.git
    kinetic-devel   # https://github.com/ros-perception/laser_filters.git
    melodic-devel   # https://github.com/ros-perception/slam_gmapping.git
    melodic-devel   # https://github.com/ros-perception/openslam_gmapping.git
    develop     # https://github.com/ros-drivers/usb_cam.git
    noetic-devel    # https://github.com/ros-perception/image_transport_plugins
    main  # https://github.com/ros/rosbag_snapshot.git
    master  # https://github.com/stereolabs/zed-ros-wrapper
    master  # https://github.com/stereolabs/zed-ros-examples
    master  # https://github.com/alireza-hosseini/ipcamera_driver
    melodic-devel  # https://github.com/ros-teleop/twist_mux.git
    melodic-devel  # https://github.com/ros-teleop/twist_mux_msgs.git
    ros1-master  # https://github.com/iralabdisco/ira_laser_tools.git
    lunar-devel  # https://github.com/ros-perception/pointcloud_to_laserscan.git
)

len=${#packages[@]}
for (( i=0; i<$len; i++ )); do
    git clone --recursive ${packages[i]} --branch ${branches[i]}
done

cd ..

rosdep install --from-paths src --ignore-src --rosdistro=noetic -y -r

if [ ! -z ${APPLY_PATCHES} ]; then
    ${BASE_DIR}/06a_apply_patches.sh
fi

cd ${DEPENDENCIES_WS}
catkin_make -j5
success=$?
if [[ $success -eq 0 ]];
then
    echo "Packages built successfully"
else
    echo "Something went wrong!"
    exit 1
fi
