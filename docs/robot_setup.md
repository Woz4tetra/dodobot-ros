
# SD card Installation

This guide assumes you're using a linux machine to run all these commands. You'll have to adapt these instructions if you're on another OS.

## From an SD card backup

### Loading a backup

[Based on this guide](https://www.jetsonhacks.com/2020/08/08/clone-sd-card-jetson-nano-and-xavier-nx/)

- [Download Jetson SD card image](https://i.kym-cdn.com/photos/images/newsfeed/002/203/505/797.png) WARNING this link doesn't work yet.
- Find disk path
- Run all commands as root: `sudo su`
- `parted -l`
  - GUI version: open "Disks"
- `umount /dev/sdd1`
  - Replace /dev/sdd1 with the partition that the OS auto mounts.
- `gunzip -c ./dodobot.img.gz | dd of=/dev/sdd bs=64K status=progress`
  - Replace /dev/sdd with the top level disk name
  - WARNING: you can very easily destroy your disk if you select the wrong one. Please use caution here.


### Creating a backup

- `sudo dd if=/dev/sdd conv=sync,noerror bs=64K status=progress | gzip -c > ./dodobot.img.gz`
  - Replace `/dev/sdd` with the top level disk name
  - WARNING: you can very easily destroy your disk if you select the wrong one. Please use caution here.

---

# Manual installation

If you don't have access to this SD card backup or need to start from scratch, follow these steps.

Unless otherwise stated, these commands are to be run on the Jetson.

I recommend running all of these commands inside of a tmux session in case of network dropouts:
- Create tmux session: `tmux new -s build`
- Hide tmux session: ctrl-B D
- Reattach tmux session: `tmux a -t build`

## Jetson initial setup

[Follow this guide from NVidia](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit)

Setup options:
- username: ben
- password: s0mething
- hostname/computer name: chansey
- Log in automatically: Yes

After rebooting, try ssh: `ssh ben@chansey.local` <br>
Install openssh server if it doesn’t work: `sudo apt-get install openssh-server -y`

## Apt refresh + basic packages
- `sudo apt update`
- `sudo apt upgrade`
- `sudo apt autoremove`
- `sudo apt install nano tmux curl htop`
- `sudo reboot`

## SSH setup

If you haven't generated new SSH keys, [follow this guide](networking/ssh_instructions.md)

If you already have a key generated:
- Upload keys to `~/.ssh` on the Jetson (make the directory if it doesn’t exist)
- `cp chansey.pub authorized_keys`
- Optionally disable password login:
  - `sudo nano /etc/ssh/sshd_config`
  - Search for `#PasswordAuthentication yes`
  - Change to `PasswordAuthentication no`
  - Save and exit (ctrl-S ctrl-X)
- `sudo service ssh restart`
- Try to login with `ssh -i ~/.ssh/chansey ben@chansey.local` (use the Jetson's IP address instead of chansey.local if that doesn't work)

## Add to sudo group
- usermod -aG sudo $USER
- sudo visudo
- If vim opens,
  - Press a to enter edit mode
  - Add the following line to the file: <br>`ben  ALL=(ALL) NOPASSWD:ALL`
  - Press esc
  - Type `:x`
  - Press enter
- If nano opens,
  - Add the following line to the file: <br>`ben  ALL=(ALL) NOPASSWD:ALL`
  - Press ctrl-S then ctrl-X
- Log out with ctrl-D
- Log in and try `sudo su`
- If no password prompt appears, these steps worked

## Upload code

Run this command on your local machine:

`~/dodobot-ros/install/upload.sh chansey.local ~/.ssh/chansey n`

## Upload firmware

### Install platformio

[Based on this guide](https://docs.platformio.org/en/latest/core/installation.html#super-quick-mac-linux)

- `python3 -c "$(curl -fsSL https://raw.githubusercontent.com/platformio/platformio/master/scripts/get-platformio.py)"`
- `sudo ln -s /home/$USER/.platformio/penv/bin/platformio /usr/local/bin`

### Main firmware upload

#### Compile

- `cd ~/dodobot-ros/firmware/main-firmware`
- `./compile.sh`
- Packages should download (~15-20 minutes)
- Platformio should say "SUCCESS"

#### Upload
- sudo apt-get install libusb-0.1-4
- `wget https://www.pjrc.com/teensy/00-teensy.rules`
- `sudo mv 00-teensy.rules /etc/udev/rules.d/49-teensy.rules`
- `sudo adduser $USER dialout`
- `sudo reboot`
- `cd ~/dodobot-ros/firmware/main-firmware`
- `./upload.sh`
- Platformio should say "SUCCESS"

### Power box firmware upload

#### Compile

- `cd ~/dodobot-ros/firmware/power-box-firmware`
- `./compile.sh`
- Packages should download (~15-20 minutes)
- Platformio should say "SUCCESS"

#### Upload
- `cd ~/dodobot-ros/firmware/power-box-firmware`
- `./upload.sh`
- Platformio should say "SUCCESS"

## dodobot_py

This package manages various systems like wifi control and the power button.

- Download [Music](https://drive.google.com/drive/u/1/folders/1SWovwBrNBWshl8dWEFh8yaypQgWAwOSE)
- Run this from your local machine. Upload Music.zip: `scp -i ~/.ssh/chansey ~/Downloads/Music.zip ben@chansey.local:/home/ben`
- `unzip Music.zip`
- `sudo apt-get install python3-pip python3-pil`
- `cd ~/dodobot-ros/dodobot_py`
- `./install.sh`

### Check if it’s running
- `systemctl status --user dodobot_py.service`
- The speaker should play a sound on successful startup
- Watch the process logs
- `tail -F -n 300 ~/.local/dodobot/dodobot_py/logs/dodobot`
- Reboot to ensure that the process starts on its own: `sudo reboot`

## ROS and Package Dependencies

I recommend running all of these commands inside of a tmux session in case of network dropouts:
- Create tmux session: `tmux new -s build`
- Hide tmux session: ctrl-B D
- Reattach tmux session: `tmux a -t build`

Download all packages to a home based directory:
- `mkdir ~/build_ws`
- `cd ~/build_ws`

Tip: list all CMake options: `cmake -LA | awk '{if(f)print} /-- Cache values/{f=1}'`

### Install TBB
- `cd ~/build_ws`
- `git clone https://github.com/wjakob/tbb.git`
- `cd tbb/build`
- `cmake ..`
- `make -j3`
- `sudo make install`

### Install numba

- `sudo apt install llvm-7*`
- `sudo ln -s /usr/lib/llvm-7/bin/llvm-config /usr/bin`
- `sudo -H pip3 install Cython`
- `sudo -H pip3 install llvmlite==0.32.0`
- `sudo -H pip3 install numba==0.49.0`

### Install PyKDL

[Based on orocos's guide](https://github.com/orocos/orocos_kinematics_dynamics/blob/master/python_orocos_kdl/INSTALL.md)

- `cd ~/build_ws`
- `git clone https://github.com/orocos/orocos_kinematics_dynamics.git`
- `cd orocos_kinematics_dynamics`
- `git submodule update --init`
<br><br>
- `cd orocos_kdl`
- `mkdir build && cd build`
- `cmake .. && make -j3`
- `sudo make install`
- `python3 -m pip install psutil`
- `cd ../../python_orocos_kdl`
- `mkdir build && cd build`
- ```
    cmake -D PYTHON_EXECUTABLE=/usr/bin/python3 \
    -D PYTHON_INCLUDE_DIR=/usr/include/python3.6 \
    -D PYTHON_LIBRARY=/usr/lib/aarch64-linux-gnu/libpython3.6m.so \
    -D PYBIND11_PYTHON_VERSION=3 ..```
- `make -j3`
- `sudo make install`
- `sudo -H pip3 install psutil`
- `sudo ldconfig`
- `python3 ../tests/PyKDLtest.py`  The tests may fail. Make sure they at least run

### Install OpenCV 4

[Based on pyimagesearch's guide](https://www.pyimagesearch.com/2018/08/15/how-to-install-opencv-4-on-ubuntu/)

- `sudo apt-get install libjpeg-dev libpng-dev libtiff-dev libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libxvidcore-dev libx264-dev libgtk-3-dev libatlas-base-dev gfortran python3-dev -y`
- cd ~/build_ws`
- `wget -O opencv.zip https://github.com/opencv/opencv/archive/refs/tags/4.4.0.zip`
- `wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/refs/tags/4.4.0.zip`
- `unzip opencv.zip && unzip opencv_contrib.zip`
- `mv opencv-4.4.0/ opencv && mv opencv_contrib-4.4.0/ opencv_contrib`
- `cd opencv && mkdir build && cd build`
- ```
  cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D WITH_CUDA=ON \
    -D ENABLE_FAST_MATH=1 \
    -D CUDA_FAST_MATH=1 \
    -D WITH_CUBLAS=1 \
    -D CUDNN_VERSION='8.0' \
    -D ENABLE_CXX11=ON \
    -D OPENCV_ENABLE_NONFREE=ON \
    -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
    -D PYTHON_EXECUTABLE=/usr/bin/python3 ..
  ```
- `make -j3 && sudo make install` This will take several hours

### Block rosdep from installing apt’s opencv
- `sudo nano /etc/apt/preferences`
- Paste the following contents:
    ```
    Package: python3-opencv
    Pin: release *
    Pin-Priority: -1

    Package: libopencv-dev
    Pin: release *
    Pin-Priority: -1
    ```

### Install librealsense

(Based on Intel's jetson guide)[https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_jetson.md]

- `sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE`
- `sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u`
- `sudo apt-get install librealsense2-utils`
- `sudo apt-get install librealsense2-dev`

### Install apriltag

- `cd ~/build_ws`
- `git clone https://github.com/AprilRobotics/apriltag.git`
- `cd apriltag && mkdir build && cd build`
- `cmake .. && make -j3 && sudo make install`

### Install RTABmap dependencies

#### Random dependencies

- `sudo apt-get update`
- `sudo apt-get install libsqlite3-dev libpcl-dev git cmake libproj-dev libqt5svg5-dev -y`

#### libg2o
- `sudo apt install libsuitesparse-dev qtdeclarative5-dev qt5-qmake libqglviewer-dev-qt5 libeigen3-dev -y`
- `cd ~/build_ws`
- `git clone https://github.com/RainerKuemmerle/g2o.git`
- `cd g2o`
- `git checkout tags/20200410_git`
- `mkdir build && cd build`
- `cmake -DBUILD_WITH_MARCH_NATIVE=OFF -DG2O_BUILD_APPS=OFF -DG2O_BUILD_EXAMPLES=OFF -DG2O_USE_OPENGL=OFF ..`
- `make -j3`
- `sudo make install`

#### gtsam
- `sudo add-apt-repository ppa:borglab/gtsam-develop`
- `sudo apt update`
- `sudo apt install libgtsam-dev libgtsam-unstable-dev -y`

#### Octomap
- `cd ~/build_ws`
- `git clone https://github.com/OctoMap/octomap`
- `cd octomap`
- `mkdir build && cd build`
- `cmake .. && make -j3`
- `sudo make install`

#### Upgrade CMake
This is to fix an issue with the RTABmap build. Fix -lCUDA_cublas_device_LIBRARY-NOTFOUND issue:
- https://github.com/clab/dynet/issues/1457
- Install CMake version 3.12.2 or higher
<br>
<br>

- `sudo apt remove --purge cmake`
- `sudo snap install cmake --classic`
- `echo "export PATH=${PATH}:/snap/bin" >> ~/.bashrc`
- Close and reopen terminal for this to take effect

### Install rtabmap
- `cd ~/build_ws`
- `git clone https://github.com/introlab/rtabmap.git`
- `cd rtabmap/build`
- `cmake .. && make -j1`  This will take a few hours
- `sudo make install`
- `sudo ldconfig`

### Install yolov5 dependencies

#### Install pytorch dependencies

- `cd ~/build_ws`
- `pip3 install "seaborn>=0.11.0" "pandas>=1.1.4" "thop" "scipy>=1.4.1" "matplotlib>=3.2.2" "tqdm"`
- `sudo apt-get install python3-pip libjpeg-dev libopenblas-dev libopenmpi-dev libomp-dev -y`
- `sudo -H pip3 install future`
- `sudo pip3 install -U --user wheel mock pillow`
- `sudo -H pip3 install testresources`
- `sudo -H pip3 install setuptools==58.3.0`
- `sudo -H pip3 install Cython`

#### Install pytorch from wheel
- `wget -O torch-1.10.0-cp36-cp36m-linux_aarch64.whl https://nvidia.box.com/shared/static/fjtbno0vpo676a25cgvuqc1wty0fkkg6.whl`
- `sudo -H pip3 install torch-1.10.0-cp36-cp36m-linux_aarch64.whl`

#### Install pytorch from source

- `git clone --recursive https://github.com/pytorch/pytorch`
- `cd pytorch`
- `git checkout v1.10.2`
- `git submodule update`
- `mkdir build && cd build`
- `export MAX_JOBS=2`
  - cmake pulls this value into build commands run by make
- `cmake -DGLIBCXX_USE_CXX11_ABI=1 -DBUILD_SHARED_LIBS:BOOL=ON -DCMAKE_BUILD_TYPE:STRING=Release -DPYTHON_EXECUTABLE:PATH=`\`which python3\` `..`
    - If you get this error: “No CMAKE_CUDA_COMPILER could be found.” add nvcc to your path:
    - `echo "export CUDACXX=/usr/local/cuda/bin/nvcc" >> ~/.bashrc`
- `make -j2 && sudo make -j2 install`

#### Install pytorch vision
- `cd ~/build_ws`
- `git clone https://github.com/pytorch/vision`
- `cd vision`
- `git checkout v0.11.2`
    - if an error like this appears during the build: <br>`‘cached_cast’ is not a member of ‘at::autocast’` <br> checkout v0.7.0 instead
- `export MAKEFLAGS="-j2"`
- `sudo python3 setup.py install`
- `echo "export CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:${HOME}/.local/lib/python3.6/site-packages/torch/share/cmake/Torch" >> ~/.bashrc`
- `sudo apt-get install python3-dev`

## ROS Installation

- `cd ~/dodobot-ros/install/source_installation`
- `./00_ros_setup.sh`
- `./01_noetic_ws_prep.sh`
- `./02_noetic_ws_rosdep.sh` This may take a while
- `./03_noetic_ws_patch.sh`
- `./04_noetic_ws_install.sh` This will take a few hours
- `./05_append_env.sh`
- Close and reopen terminal
- `./06_build_packages_ws.sh y` y indicates we want to apply the patches. This will take a few hours
- Close and reopen terminal
- `./07_prep_ros_ws.sh`
- `./08_build_ros_ws.sh`
- `./09_install_python_libraries.sh`

## dodobot-ros systemd install
- `cd ~/dodobot-ros/systemd`
- `sudo ./install_systemd.sh`
- Verify installation:
    - `sudo systemctl status roscore.service`
    - `sudo systemctl status roslaunch.service`

## Xbox joystick
[Based on this guide](https://pimylifeup.com/xbox-controllers-raspberry-pi/)

[and this one](https://forums.developer.nvidia.com/t/disabling-ertm-permanently-in-jetpack-4-4-ubuntu-18-04-on-nano-4gb/159567)

(wired only)
- `sudo apt-get install xboxdrv sysfsutils`
- `sudo reboot`
- `sudo apt-get install joystick`
- `sudo jstest /dev/input/js0`


## Hotspot
Requires a secondary wifi adapter (USB is ok)

### TP-Link AC1300 Archer T3U setup

[Based on this post](https://www.forecr.io/blogs/connectivity/tp-link-ac600-ac1300-archer-t2u-t3u-driver-installation-for-jetson-modules)

- Check if the device is connected: `lsusb`
  - An entry should appear and disappear when you plug and unplug it
- `sudo apt install git dkms`
- `cd ~/build_ws`
- `https://github.com/morrownr/88x2bu-20210702.git`
- `cd 88x2bu-20210702`
- `./ARM64_RPI.sh`
- `sudo ./install-driver.sh`

### Hotspot setup

- `sudo apt-get update`
- `sudo apt-get install hostapd dnsmasq   --assume-yes`
- `sudo systemctl stop dnsmasq`
- `sudo systemctl stop hostapd`

Copy the following files:
- `sudo su`
- `cd ~/dodobot-ros/networking`
- `cp dhcpcd.conf /etc/dhcpcd.conf`
- `cp dnsmasq.conf /etc/dnsmasq.conf`
- `cp hostapd.conf /etc/hostapd/hostapd.conf`
- `cp hostapd /etc/default/hostapd`
- `cp sysctl.conf /etc/sysctl.conf`
- `cp interfaces /etc/network/interfaces`
- `cp rc.local /etc/rc.local`
- `chmod +x /etc/rc.local`

Set routing from hotspot to internet connection:
- `sudo su`
- `iptables -A FORWARD -i wlan1 -o wlan0 -j ACCEPT`
- `iptables -A FORWARD -i wlan0 -o wlan1 -m state --state ESTABLISHED,RELATED -j ACCEPT`
- `iptables -t nat -A  POSTROUTING -o wlan0 -j MASQUERADE`
- `sh -c "iptables-save > /etc/iptables.ipv4.nat"`

Stop network manager from controlling hotspot (copy file into place).
<br>
[Helpful forum post](https://askubuntu.com/questions/472794/hostapd-error-nl80211-could-not-configure-driver-mode)
- `ifconfig`
- Look for the hotspot device’s MAC address:
```
wlan0: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
        inet 192.168.0.57  netmask 255.255.255.0  broadcast 192.168.0.255
        inet6 xxxx::xxxx:xxxx:xxxx:xxxx  prefixlen 64  scopeid 0x20<link>
        inet6 xxxx:xxx:xxxx:xxxx:xxxx:xxxx:xxxx:xxxx  prefixlen 64  scopeid 0x0<global>
        inet6 xxxx:xxx:xxxx:xxxx:xxxx:xxxx:xxx:xxxx  prefixlen 64  scopeid 0x0<global>
        ether XX:XX:XX:XX:XX:XX  txqueuelen 1000  (Ethernet)
        RX packets 1309926  bytes 1898774284 (1.8 GB)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 198667  bytes 49756412 (49.7 MB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0
```
- In this case. See this line: `ether XX:XX:XX:XX:XX:XX`
- `cd ~/dodobot-ros/networking`
- `sudo cp NetworkManager.conf /etc/NetworkManager/NetworkManager.conf`
- `sudo nano /etc/NetworkManager/NetworkManager.conf`
- Replace the x’s in mac:xx:xx:xx:xx:xx:xx with the MAC address of your device

Apply NetworkManager changes and start access point (do while connected to a display):
- `sudo rfkill unblock wlan`
- `sudo systemctl restart NetworkManager.service`
- `sudo ifconfig wlan0 up`
- `sudo ifconfig wlan1 up`
- `sudo service dnsmasq restart`
- `sudo service hostapd restart`
- `sudo nmcli radio wifi on`

Set persistent interface names:
- Get MAC addresses for wlan0 and wlan1: `ifconfig`
- `sudo nano /etc/udev/rules.d/70-persistent-net.rules`
- Fill with wlan0 and wlan1:
```
# interface with MAC address "XX:XX:XX:XX:XX:XX" will be assigned "wlan0"
SUBSYSTEM=="net", ACTION=="add", DRIVERS=="?*", ATTR{address}=="XX:XX:XX:XX:XX:XX", ATTR{dev_id}=="0x0", ATTR{type}=="1", KERNEL=="wlan*", NAME="wlan0"

# interface with MAC address "XX:XX:XX:XX:XX:XX" will be assigned "wlan1"
SUBSYSTEM=="net", ACTION=="add", DRIVERS=="?*", ATTR{address}=="XX:XX:XX:XX:XX:XX", ATTR{dev_id}=="0x0", ATTR{type}=="1", KERNEL=="wlan*", NAME="wlan1"
```
