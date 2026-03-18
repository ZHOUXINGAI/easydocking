# QGroundControl 配置指南

## 连接配置

### 1. 启动QGC
```bash
# 下载QGC
wget https://github.com/mavlink/qgroundcontrol/releases/download/v4.2.8/QGroundControl.AppImage
chmod +x QGroundControl.AppImage

# 启动
./QGroundControl.AppImage
```

### 2. 配置通信连接

在QGC中配置两个UDP连接：

#### Carrier (母舰) - UDP 1
- **Name**: Carrier
- **Type**: UDP
- **Host**: 127.0.0.1
- **Port**: 14580
- **Server Address**: 127.0.0.1:14580

#### Mini (子机) - UDP 2
- **Name**: Mini
- **Type**: UDP
- **Host**: 127.0.0.1
- **Port**: 14581
- **Server Address**: 127.0.0.1:14581

### 3. PX4启动参数

启动PX4 SITL时自动配置MAVLink端口：

```bash
# Carrier
PX4_SYS_AUTOSTART=4001 \
PX4_GZ_MODEL_POSE="0,0,0,0,0,0" \
./build/px4_sitl_default/bin/px4 -i 1

# Mini
PX4_SYS_AUTOSTART=4001 \
PX4_GZ_MODEL_POSE="10,0,0,0,0,0" \
./build/px4_sitl_default/bin/px4 -i 2
```

MAVLink自动配置：
- Instance 1 (Carrier): UDP 14580
- Instance 2 (Mini): UDP 14581

### 4. 地面站显示

QGC将显示：
- 两架无人机的位置
- 飞行状态
- 电池电量
- 飞行模式
- 任务进度

## 常用操作

### 解锁/上锁
1. 选择Carrier或Mini
2. 点击"解锁"按钮
3. 确认解锁

### 起飞
1. 设置起飞高度
2. 点击"起飞"按钮

### 降落
1. 点击"降落"按钮
2. 或切换到"降落"模式

### 任务规划
1. 在地图上点击设置航点
2. 上传任务到无人机
3. 启动任务

## 对接任务监控

### 观察指标
- **相对距离**: 在RViz中查看
- **对接状态**: 在ROS2话题 `/docking/status` 中
- **阶段**: APPROACH → TRACKING → DOCKING → COMPLETED

### 手动干预
如需手动控制：
1. 在QGC中选择对应的无人机
2. 切换到"Position"或"Offboard"模式
3. 使用摇杆或键盘控制

## 故障排查

### 无法连接
1. 检查PX4是否启动
2. 检查UDP端口是否正确
3. 检查防火墙设置

### 数据显示异常
1. 检查MicroXRCEAgent是否运行
2. 检查ROS2话题是否有数据
3. 检查坐标系设置

## 配置文件

QGC配置文件位置：
```
~/.config/QGroundControl.org/
```

可以保存和加载不同的配置。
