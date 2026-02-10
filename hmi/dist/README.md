# FlexMind Project README

## 项目概览
该工程是一个基于 ROS1 (Noetic) 的双臂机器人控制系统，包含以下核心能力：
- 双臂机器人实时跟随控制
- 夹爪控制与状态反馈
- 相机数据接入（RealSense + Orbbec）
- 远程操作（GELLO）与锁定/标定
- 录制数据（rosbag/mcap）
- 前端通信服务（rosbridge + web_video_server）

当前工作区主要代码位于 `fleximind-ros1-release-1.0.0/`，根目录还包含前端打包产物压缩包（`dist.zip`、`flexmind-node.zip`）。

## 目录结构
```text
FlexMind/
├─ fleximind-ros1-release-1.0.0/   # ROS1 主工程
│  ├─ src/
│  │  ├─ fleximind_bringup
│  │  ├─ fleximind_interaction
│  │  ├─ fleximind_hardware
│  │  ├─ fleximind_remote
│  │  ├─ fleximind_robot
│  │  ├─ fleximind_gripper
│  │  ├─ fleximind_sensors
│  │  ├─ realsense-ros
│  │  ├─ OrbbecSDK_ROS1
│  │  └─ web_video_server
│  ├─ build.sh
│  ├─ run.sh
│  ├─ requirements.txt
│  └─ docs/
└─ dist.zip / flexmind-node.zip
```

## 运行环境
- Ubuntu 20.04
- ROS Noetic
- Python 3.8
- catkin tools (`catkin_make`)

## 快速开始
```bash
# 1) 进入工作区（示例）
cd /home/robot
# 如果当前目录名是 fleximind-ros1-release-1.0.0，建议改名
mv fleximind-ros1-release-1.0.0 fleximind-ros1
cd fleximind-ros1

# 2) 安装 ROS 依赖
sudo apt update
sudo apt install ros-noetic-desktop
sudo apt install ros-noetic-rosbridge-suite
sudo apt install ros-noetic-web-video-server
sudo apt install ros-noetic-ddynamic-reconfigure
sudo apt install ros-noetic-librealsense2
sudo apt install ros-noetic-realsense2-camera

# 3) 安装 Orbbec 相关依赖
sudo apt install libgflags-dev ros-noetic-image-geometry ros-noetic-camera-info-manager \
  ros-noetic-image-transport-plugins ros-noetic-compressed-image-transport \
  ros-noetic-image-transport ros-noetic-image-publisher libgoogle-glog-dev \
  libusb-1.0-0-dev libeigen3-dev ros-noetic-diagnostic-updater \
  ros-noetic-diagnostic-msgs libdw-dev
sudo apt install libxml2-dev libxslt-dev

# 4) Python 虚拟环境
python3 -m venv venv38 --system-site-packages --symlinks
source venv38/bin/activate
pip install --upgrade pip
pip install -r src/fleximind_remote/src/fleximind_remote/requirements.txt
pip install src/fleximind_remote/src/fleximind_remote/third_party/DynamixelSDK/python
pip install -r requirements.txt

# 5) 编译
source /opt/ros/noetic/setup.bash
./build.sh
source devel/setup.bash
```

## 启动系统
```bash
# 方式1：脚本启动（推荐）
./run.sh

# 方式2：手动启动
source venv38/bin/activate
source /opt/ros/noetic/setup.bash
source devel/setup.sh
roslaunch fleximind_bringup bringup.launch
```

`bringup.launch` 会拉起以下模块：
- interaction: 主交互节点、rosbridge、web_video_server
- hardware: camera 驱动、硬件监控、安全监控、配置管理
- remote: 远程控制节点与系统状态节点
- gripper: 夹爪控制节点
- robot: 双臂机器人控制节点
- sensors: 录制服务

## 默认通信端口
- rosbridge websocket: `9090`
- web_video_server: `8080`

## 关键配置文件
- 远程控制配置: `src/fleximind_remote/config/remote.yaml`
- 机器人配置: `src/fleximind_robot/config/config.yaml`
- 夹爪配置: `src/fleximind_gripper/config/gripper_config.yaml`
- 录制配置: `src/fleximind_sensors/config/record.yaml`
- 相机配置与序列号: `src/fleximind_hardware/launch/cameras.launch`

## 前端部署
当前仓库根目录已经提供了前端与中间层服务包：
- 前端静态资源: `dist.zip`
- Node 中间层服务: `flexmind-node.zip`

### 方案 A：Node 一体化部署（最快）
适合快速上线调试，Node 同时托管前端静态文件和 API。

```bash
# 1) 准备目录
sudo mkdir -p /opt/flexmind/frontend /opt/flexmind/flexmind-node

# 2) 解压前端静态文件
sudo unzip dist.zip -d /opt/flexmind/frontend

# 3) 解压 Node 服务
sudo unzip flexmind-node.zip -d /opt/flexmind/flexmind-node

# 4) 用前端构建产物覆盖 Node 的 public 目录
sudo rm -rf /opt/flexmind/flexmind-node/public/*
sudo cp -a /opt/flexmind/frontend/dist/. /opt/flexmind/flexmind-node/public/

# 5) 安装并启动 Node 服务
cd /opt/flexmind/flexmind-node
npm install
PORT=3001 ROSBRIDGE_URL=ws://127.0.0.1:9090 CORS_ORIGIN=http://<服务器IP>:3001 npm run start
```

启动后访问：
```text
http://<服务器IP>:3001
```

### 方案 B：Nginx + Node（推荐生产）
Nginx 托管前端静态文件，`/ros`、`/playback_files`、`/socket.io` 转发到 Node。

1. Node 服务启动（示例）：
```bash
cd /opt/flexmind/flexmind-node
npm install
PORT=3001 ROSBRIDGE_URL=ws://127.0.0.1:9090 CORS_ORIGIN=http://<服务器IP> npm run start
```

2. Nginx 站点配置（示例）：
```nginx
server {
    listen 80;
    server_name _;

    root /opt/flexmind/frontend/dist;
    index index.html;

    location / {
        try_files $uri $uri/ /index.html;
    }

    location /ros/ {
        proxy_pass http://127.0.0.1:3001;
        proxy_http_version 1.1;
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
    }

    location /playback_files/ {
        proxy_pass http://127.0.0.1:3001;
        proxy_http_version 1.1;
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
    }

    location = /playbackService {
        proxy_pass http://127.0.0.1:3001;
        proxy_http_version 1.1;
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
    }

    location /socket.io/ {
        proxy_pass http://127.0.0.1:3001;
        proxy_http_version 1.1;
        proxy_set_header Upgrade $http_upgrade;
        proxy_set_header Connection "upgrade";
        proxy_set_header Host $host;
    }
}
```

3. 应用配置并重载：
```bash
sudo nginx -t
sudo systemctl reload nginx
```

### 前端联通性要求（重要）
- 浏览器必须能访问 ROSBridge：`ws://<服务器IP>:9090`
- Node 服务默认端口：`3001`
- 使用 Nginx 时，前端页面通常走 `80` 端口

如果 `9090` 端口被防火墙拦截，页面中的实时状态（部分直接走 rosbridge 的 WebSocket）会失败。

## 自动启动部署（systemd）
```bash
cd /home/robot/fleximind-ros1
sudo cp ./docs/fleximind-ros1.service /etc/systemd/system/
sudo cp ./docs/hotpot.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable fleximind-ros1.service
sudo systemctl restart fleximind-ros1.service
sudo systemctl enable hotpot.service
sudo systemctl restart hotpot.service
```

## udev 规则（串口设备）
```bash
cd /home/robot/fleximind-ros1
sudo cp ./docs/50-ftdi-ft232h.rules /etc/udev/rules.d/50-ftdi-ft232h.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```
