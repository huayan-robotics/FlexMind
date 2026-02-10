# FlexiMind-ROS1

## Getting started
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt update
sudo apt install ros-noetic-desktop
sudo apt install ros-noetic-rosbridge-suite
sudo apt install ros-noetic-web-video-server
sudo apt install ros-noetic-ddynamic-reconfigure

sudo apt install ros-noetic-librealsense2
sudo apt install ros-noetic-realsense2-camera

# Orbbec Install
sudo apt install libgflags-dev ros-noetic-image-geometry ros-noetic-camera-info-manager \
ros-noetic-image-transport-plugins ros-noetic-compressed-image-transport \
ros-noetic-image-transport ros-noetic-image-publisher libgoogle-glog-dev libusb-1.0-0-dev libeigen3-dev \
ros-noetic-diagnostic-updater ros-noetic-diagnostic-msgs \
libdw-dev

sudo apt install libxml2-dev libxslt-dev
sudo apt install python3-venv
python3 -m venv venv38 --system-site-packages --symlinks
source venv38/bin/activate
pip install --upgrade pip
pip install -r src/fleximind_remote/src/fleximind_remote/requirements.txt
pip install src/fleximind_remote/src/fleximind_remote/third_party/DynamixelSDK/python
source /opt/ros/noetic/setup.bash
./build.sh
source ./devel/setup.bash
pip install -r requirements.txt

cd ~/fleximind-ro1
roscd orbbec_camera
sudo bash ./scripts/install_udev_rules.sh
```

## Install
```
catkin_make install
```

## Deploy

```
# 删除可能冲突的旧规则
sudo rm -f /etc/udev/rules.d/99-ftdi-serial.rules
sudo rm -f /etc/udev/rules.d/99-serial.rules

# 创建新的udev规则文件
sudo cp ./docs/50-ftdi-ft232h.rules /etc/udev/rules.d/50-ftdi-ft232h.rules

# 重新加载udev规则
sudo udevadm control --reload-rules
sudo udevadm trigger

# 重启udev服务
sudo systemctl restart udev

# 重新插拔USB设备，或者运行：
sudo udevadm trigger --subsystem-match=tty --action=add

# 以上配置可能导致相机无法启动需要重新执行奥比usb策略脚本！！！

# 自启服务
sudo cp ./docs/fleximind-ros1.service /etc/systemd/system/
sudo cp ./docs/hotpot.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable fleximind-ros1.service
sudo systemctl restart fleximind-ros1.service
sudo systemctl enable hotpot.service
sudo systemctl restart hotpot.service
```

## Create Package

```
# eg.
catkin_create_pkg fleximind_remote std_msgs rospy roscpp

# This is an example, do not try to run this
catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
```


## Name
Fleximind Platform for ROS1 version

## Description
For bimanual robots platform

## Authors and acknowledgment
Lianzr

## License
All copyrights by HuaYan Robotics.
