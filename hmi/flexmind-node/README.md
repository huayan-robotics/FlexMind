# FlexiMind-Node

## 简介
这是一个基于 Koa 的 Node.js Web 应用示例，包含基础中间件、路由、错误处理、日志、静态文件托管、配置管理等。

## 目录结构
```text
koa-rosbridge-socketio/
├── public/
│   └── index.html             # 前端示例页面
├── src/
│   ├── index.js               # 启动入口：创建 HTTP + Socket.IO Server，并初始化 rosBridgeService
│   ├── app.js                 # Koa 应用：挂载中间件 & HTTP 路由（可选）
│   ├── middleware/
│   │   ├── logger.js          # 简单日志中间件
│   │   └── error-handler.js     # 全局错误处理中间件
│   ├── services/
│   │   └── rosBridgeService.js  # rosbridge 客户端封装：connect, subscribe, publish, callService...
│   ├── routes/
│   │   └── ros-routes.js        # (可选) HTTP 路由示例：通过 HTTP 发布/查询
│   └── utils/
│       └── responseHelper.js  # (可选) 统一 HTTP 响应格式
├── config/
│   └── default.js             # 读取 .env 配置（rosbridge URL、port 等）
├── .env.example               # 环境变量示例
├── package.json
├── start.sh                   # 启动脚本：可选，用于设置环境变量再启动 Node 应用
└── README.md
```

## 环境要求
- Node.js 版本 >= 18.0.0
- npm 版本 >= 9.0.0

## 安装依赖

```bash
npm install
```

## 环境配置

1. 复制环境变量示例文件：
```bash
cp .env.example .env
```

2. 编辑 `.env` 文件，配置必要的环境变量：
```bash
# 服务端口
PORT=3000

# ROS Bridge WebSocket 地址
ROSBRIDGE_URL=ws://localhost:9090

# 其他配置...
```

## 开发部署步骤

### 本地开发

使用 nodemon 启动开发服务器，支持热重载：

```bash
npm run dev
```

开发服务器将在 `http://localhost:3000` 启动（端口可在 .env 中配置）

### 生产部署

#### 方法一：直接运行

```bash
npm start
```

#### 方法二：使用启动脚本

```bash
bash start.sh
```

#### 方法三：使用 PM2 进程管理（推荐）

1. 全局安装 PM2：
```bash
npm install -g pm2
```

2. 启动应用：
```bash
pm2 start src/index.js --name fleximind-node
```

3. 常用 PM2 命令：
```bash
# 查看应用状态
pm2 status

# 查看日志
pm2 logs fleximind-node

# 重启应用
pm2 restart fleximind-node

# 停止应用
pm2 stop fleximind-node

# 设置开机自启
pm2 startup
pm2 save
```

## 代码规范

### 代码检查和格式化

```bash
# ESLint 检查并自动修复
npm run lint

# Prettier 格式化所有文件
npm run lint:prettier

# Stylelint 格式化样式文件
npm run lint:stylelint

# 仅格式化 src 目录
npm run format
```

## 常用命令

| 命令 | 说明 |
|------|------|
| `npm run dev` | 启动开发服务器（热重载） |
| `npm start` | 启动生产服务器 |
| `npm run lint` | 代码检查 |
| `npm run format` | 代码格式化 |
| `npm test` | 运行测试 |

## 访问应用

- 前端页面: `http://localhost:3000`
- 静态文件: `http://localhost:3000/public/`

## 注意事项

1. 确保 ROS Bridge 服务已启动并可访问
2. 确保 `.env` 文件中的配置正确
3. 生产环境建议使用 PM2 或其他进程管理工具
4. 定期更新依赖包以获取安全更新
