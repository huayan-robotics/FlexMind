# FlexMind 部署说明（前后端）

本仓库包含三个主要目录：

- `flexmind-ros1-release-1.0.0`：ROS1 后端（机器人控制、硬件、传感、交互）
- `hmi`：人机界面相关发布物（Node.js 中间层 + 前端静态构建）
- `docs`：项目文档（PDF 等）

本文档提供一套完整的前后端部署流程（Ubuntu 20.04 + ROS Noetic）。

## 相关开源项目

- `dual_arm`：双臂机器人相关开源项目：`https://github.com/huayan-robotics/dual_arm`

## 1. 系统架构

运行链路如下：

1. 前端（`hmi/dist` 构建产物）访问 Node API：`http://<服务器IP>:3001`
2. 前端直接连接 rosbridge：`ws://<服务器IP>:9090`
3. Node 服务调用 rosbridge（默认也是 `ws://<服务器IP>:9090`）
4. ROS1 后端同时提供：
   - rosbridge websocket：`9090`
   - web_video_server：`8080`

注意：当前 `dist` 构建产物中，前端 API 地址是固定逻辑 `http://${window.location.hostname}:3001`，不是相对路径。

## 2. 目录与发布包

```text
FlexMind/
├─ docs/
│  └─ FlexMind数据采集平台V1.0.pdf
├─ hmi/
│  ├─ dist/
│  │  ├─ dist.zip
│  │  └─ README.md
│  └─ flexmind-node/
│     ├─ flexmind-node.zip
│     └─ README.md
├─ flexmind-ros1-release-1.0.0/
│  ├─ build.sh
│  ├─ run.sh
│  ├─ requirements.txt
│  ├─ docs/
│  └─ src/
└─ README.md
```

## 3. 部署前准备

## 3.1 推荐环境

- Ubuntu 20.04
- ROS Noetic
- Python 3.8（venv）
- Node.js >= 18，npm >= 9

## 3.2 端口放行

至少确认以下端口可访问：

- `3001`：Node API/Socket 服务
- `9090`：rosbridge websocket（前端直连）
- `8080`：web_video_server（视频流）
- `80`：如使用 Nginx 托管前端

## 3.3 路径约束（重要）

`flexmind-ros1-release-1.0.0/build.sh` 默认使用：

```bash
~/fleximind-ros1/venv38/bin/python
```

建议将 ROS 工作目录命名为 `/home/robot/fleximind-ros1`，否则需手动修改 `build.sh` 中的 Python 路径。

## 4. ROS1 后端部署（flexmind-ros1）

以下步骤在目标机器人主机执行。

## 4.1 准备代码

```bash
cd /home/robot
cp -r /path/to/FlexMind/flexmind-ros1-release-1.0.0 ./fleximind-ros1
cd /home/robot/fleximind-ros1
```

## 4.2 安装 ROS 与系统依赖

```bash
sudo apt update
sudo apt install -y ros-noetic-desktop
sudo apt install -y ros-noetic-rosbridge-suite
sudo apt install -y ros-noetic-web-video-server
sudo apt install -y ros-noetic-ddynamic-reconfigure
sudo apt install -y ros-noetic-librealsense2
sudo apt install -y ros-noetic-realsense2-camera

sudo apt install -y \
  libgflags-dev ros-noetic-image-geometry ros-noetic-camera-info-manager \
  ros-noetic-image-transport-plugins ros-noetic-compressed-image-transport \
  ros-noetic-image-transport ros-noetic-image-publisher libgoogle-glog-dev \
  libusb-1.0-0-dev libeigen3-dev ros-noetic-diagnostic-updater \
  ros-noetic-diagnostic-msgs libdw-dev libxml2-dev libxslt-dev \
  python3-venv zip
```

说明：`zip` 用于 Node 回放文件导出接口（`/playback_files/export_file`）。

## 4.3 Python 虚拟环境与依赖

```bash
cd /home/robot/fleximind-ros1
python3 -m venv venv38 --system-site-packages --symlinks
source venv38/bin/activate
pip install --upgrade pip
pip install -r src/fleximind_remote/src/fleximind_remote/requirements.txt
pip install src/fleximind_remote/src/fleximind_remote/third_party/DynamixelSDK/python
pip install -r requirements.txt
```

## 4.4 编译

```bash
cd /home/robot/fleximind-ros1
source /opt/ros/noetic/setup.bash
./build.sh

# 如需全量清理后重编译
# ./build.sh -c
```

## 4.5 启动后端

```bash
cd /home/robot/fleximind-ros1
./run.sh
```

`run.sh` 会执行：

1. `source venv38/bin/activate`
2. `source /opt/ros/noetic/setup.bash`
3. `source devel/setup.sh`
4. `roslaunch fleximind_bringup bringup.launch`

`bringup.launch` 会包含交互、相机、硬件监控、传感、远程控制、夹爪、机器人等模块，并启动 rosbridge（9090）与 web_video_server（8080）。

## 5. Node 中间层部署（flexmind-node）

## 5.1 解压与安装

```bash
sudo mkdir -p /opt/flexmind
sudo unzip /path/to/FlexMind/hmi/flexmind-node/flexmind-node.zip -d /opt/flexmind/flexmind-node
cd /opt/flexmind/flexmind-node
npm install
```

## 5.2 环境变量

可通过导出环境变量启动，示例：

```bash
export NODE_ENV=production
export PORT=3001
export ROSBRIDGE_URL=ws://127.0.0.1:9090
export CORS_ORIGIN=http://<服务器IP>:3001
export SOCKETIO_CORS_ORIGIN=*
npm run start
```

可选变量（来自 `config/default.js`）：

- `PORT`（默认 `3001`）
- `ROSBRIDGE_URL`（默认 `ws://<本机IP>:9090`）
- `CORS_ORIGIN`（默认 `http://<本机IP>:5174`）
- `SOCKETIO_CORS_ORIGIN`（默认 `*`）
- `FRONTEND_PORT`、`ROSBRIDGE_PORT`

## 5.3 接口健康检查

```bash
curl http://127.0.0.1:3001/health
curl http://127.0.0.1:3001/ros/health
```

## 6. 前端部署（hmi/dist）

## 6.1 方案 A：Node 一体化托管（推荐，最简单）

将前端静态文件复制到 Node 的 `public` 目录：

```bash
sudo unzip /path/to/FlexMind/hmi/dist/dist.zip -d /opt/flexmind/frontend
sudo rm -rf /opt/flexmind/flexmind-node/public/*
sudo cp -a /opt/flexmind/frontend/dist/. /opt/flexmind/flexmind-node/public/
```

然后启动 Node（见第 5 章），浏览器访问：

```text
http://<服务器IP>:3001
```

## 6.2 方案 B：Nginx 托管前端静态资源

```bash
sudo apt install -y nginx
sudo mkdir -p /opt/flexmind/frontend
sudo unzip /path/to/FlexMind/hmi/dist/dist.zip -d /opt/flexmind/frontend
```

Nginx 配置示例（`/etc/nginx/sites-available/flexmind.conf`）：

```nginx
server {
    listen 80;
    server_name _;

    root /opt/flexmind/frontend/dist;
    index index.html;

    location / {
        try_files $uri $uri/ /index.html;
    }
}
```

启用配置：

```bash
sudo ln -sf /etc/nginx/sites-available/flexmind.conf /etc/nginx/sites-enabled/flexmind.conf
sudo nginx -t
sudo systemctl reload nginx
```

注意：即使前端由 Nginx 的 `80` 端口托管，当前构建产物仍会请求 `http://<host>:3001` 与 `ws://<host>:9090`。因此 `3001` 和 `9090` 仍需对浏览器可达。

## 7. 开机自启（可选）

## 7.1 ROS 后端 systemd

项目已提供：

- `flexmind-ros1-release-1.0.0/docs/fleximind-ros1.service`
- `flexmind-ros1-release-1.0.0/docs/hotpot.service`

安装示例：

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

## 7.2 Node 服务 systemd（示例）

新建 `/etc/systemd/system/flexmind-node.service`：

```ini
[Unit]
Description=FlexMind Node Service
After=network.target

[Service]
Type=simple
User=robot
WorkingDirectory=/opt/flexmind/flexmind-node
Environment=NODE_ENV=production
Environment=PORT=3001
Environment=ROSBRIDGE_URL=ws://127.0.0.1:9090
Environment=CORS_ORIGIN=http://<服务器IP>:3001
ExecStart=/usr/bin/node src/index.js
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

启用：

```bash
sudo systemctl daemon-reload
sudo systemctl enable flexmind-node.service
sudo systemctl restart flexmind-node.service
```

## 8. 联调验收清单

## 8.1 进程与端口

```bash
ss -lntp | egrep '3001|8080|9090'
```

预期看到：

- Node 监听 `0.0.0.0:3001`
- rosbridge 监听 `0.0.0.0:9090`（或空地址绑定全网卡）
- web_video_server 监听 `0.0.0.0:8080`

## 8.2 健康检查

```bash
curl http://127.0.0.1:3001/health
curl http://127.0.0.1:3001/ros/health
```

## 8.3 浏览器验证

1. 打开 `http://<服务器IP>:3001`（或 Nginx 模式下 `http://<服务器IP>`）
2. 页面能正常加载
3. 实时状态可刷新（依赖 `ws://<服务器IP>:9090` 可连通）

## 9. 常见问题

## 9.1 页面打开但数据全空白

优先排查：

1. 浏览器是否能访问 `ws://<服务器IP>:9090`
2. Node 是否正常连接 rosbridge（看 Node 日志）
3. 防火墙是否拦截 `3001/9090`

## 9.2 Node 启动正常但导出回放失败

`/playback_files/export_file` 依赖系统 `zip` 命令，确认已安装：

```bash
which zip
```

## 9.3 编译时报 Python 路径错误

修改 `build.sh` 中 `-DPYTHON_EXECUTABLE` 路径，指向实际 `venv38/bin/python`。

## 10. 版权与声明

- 本项目版权归华沿机器人所有。
- 未经授权请勿分发商用。
