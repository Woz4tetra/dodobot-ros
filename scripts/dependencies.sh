packages=(
serial
joystick-drivers
geometry2
navigation
teb-local-planner
rtabmap-ros
realsense2-camera
)

package_list=""
for p in "${packages[@]}"; do
    package_list+="ros-melodic-$p "
done

sudo apt install $package_list
