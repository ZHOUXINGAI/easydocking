# EasyDocking

基于论文 *Aerial Autonomous Docking Control Method of Quadrotor UAV* 的三阶段空中对接复现工程。

当前实现采用：

- 飞控: PX4 Offboard
- 通信: ROS 2 Jazzy
- 可视化: RViz 2
- 控制方法: 接近 `Approach` -> 追踪 `Tracking` -> 对接 `Docking`

## 当前仓库包含什么

- `easydocking_control`
  - 三阶段对接控制器
  - 相对位姿估计
  - 追踪阶段制导律
  - 对接阶段自适应反步控制
  - ROS 2 控制节点
  - RViz 可视化节点
  - PX4 offboard bridge 脚本
- `easydocking_msgs`
  - 对接状态、相对位姿、命令消息
- `px4_msgs`
  - 已拉入工作区，可直接和 PX4 ROS 2 通信链配合使用

## 已安装到本机的内容

- `/home/hw/PX4-Autopilot`
- `/home/hw/easydocking/src/px4_msgs`
- `/home/hw/uxrce_agent_ws/install/microxrcedds_agent`

说明：

- 这台机器没有免密 `sudo`，所以我没有办法替你执行 `apt install`。
- 但仓库级源码、`px4_msgs`、`MicroXRCEAgent` 和本项目代码已经帮你拉取并构建到本地。
- 如果后续 PX4 SITL 构建缺系统依赖，只需要补最少量的 `apt` 包即可。

## 控制链路

1. `/carrier/odom` 和 `/mini/odom` 输入到对接控制器
2. 控制器输出
   - `/carrier/setpoint/pose`
   - `/carrier/setpoint/velocity`
   - `/mini/setpoint/pose`
   - `/mini/setpoint/velocity`
   - `/docking/relative_pose`
   - `/docking/status`
3. `px4_offboard_bridge.py` 将设定值转换到 PX4 `OffboardControlMode / TrajectorySetpoint / VehicleCommand`
4. `rviz_visualizer.py` 发布 TF、Path、Marker 供 RViz 使用

## 构建

```bash
cd /home/hw/easydocking
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

如需单独使用 XRCE Agent：

```bash
source /home/hw/uxrce_agent_ws/install/local_setup.bash
/home/hw/uxrce_agent_ws/install/microxrcedds_agent/bin/MicroXRCEAgent udp4 -p 8888
```

## 启动

先加载环境：

```bash
source /home/hw/easydocking/scripts/setup_local_env.sh
```

启动 ROS 2 控制和 RViz：

```bash
cd /home/hw/easydocking
./scripts/start_simulation.sh
```

如果你已经有 PX4 SITL 或真机 ROS 2 桥接在跑，也可以直接：

```bash
ros2 launch easydocking_control docking.launch.py start_px4_bridge:=true
```

启动对接任务：

```bash
ros2 topic pub --once /docking/command easydocking_msgs/msg/DockingCommand "{command: 'START'}"
```

停止：

```bash
ros2 topic pub --once /docking/command easydocking_msgs/msg/DockingCommand "{command: 'STOP'}"
```

## RViz 中可见内容

- `/carrier/pose`
- `/mini/pose`
- `/mini/trajectory`
- `/docking_zone_marker`
- `/relative_distance_marker`

RViz 配置文件位于 [config/docking.rviz](/home/hw/easydocking/config/docking.rviz)。

## 关键文件

- 控制节点: [src/easydocking_control/src/docking_controller_node.cpp](/home/hw/easydocking/src/easydocking_control/src/docking_controller_node.cpp)
- 控制器: [src/easydocking_control/src/docking_controller.cpp](/home/hw/easydocking/src/easydocking_control/src/docking_controller.cpp)
- RViz 可视化: [src/easydocking_control/scripts/rviz_visualizer.py](/home/hw/easydocking/src/easydocking_control/scripts/rviz_visualizer.py)
- PX4 bridge: [src/easydocking_control/scripts/px4_offboard_bridge.py](/home/hw/easydocking/src/easydocking_control/scripts/px4_offboard_bridge.py)
- 启动脚本: [scripts/start_simulation.sh](/home/hw/easydocking/scripts/start_simulation.sh)

## 还差的最后一步

如果你要在这台机器上把 PX4 SITL 也彻底编译到可运行，通常还需要系统级依赖。由于这里不能代你输 sudo 密码，我没有完成那一步。等你补完系统包后，推荐在 `/home/hw/PX4-Autopilot` 下执行：

```bash
make px4_sitl
```
