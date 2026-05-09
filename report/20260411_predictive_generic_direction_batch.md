# Predictive 抢位泛化验证（非硬猜方向）

时间：2026-04-11

## 验证目标

- 验证 carrier 抢位方向不依赖“固定东南”等全局硬编码。
- 在不同轨迹几何下，仍能保持同向接入并完成对接。

## 本轮关键改动

- `px4_offboard_bridge.py`：同向守卫升级为“未来轨迹驱动”的抢位方向（自适应预测时域 + 加速度外推 + 反向保护 + force最小前推）。
- `wait_for_docking_window.py`：新增 `rear_entry_energy_allow_max_relaxation`，在 prediction gate 模式下超过能量上限窗口可自动放宽（避免换轨迹时长期卡窗）。

## 结果总览

- 基线预设（5 组）：`20260411_193650/193941/194226/194510/194802` 全部 `final-pass`。
- 变轨迹 A（center=`-30,25` phase=`30` out_angle=`45`）：`20260411_210946` `final-pass`。
- 变轨迹 B（center=`42,18` phase=`260` out_angle=`160`）：`20260411_211507` `final-pass`。
- 合计：7/7 `final-pass`。

## 方向与起步指标（carrier_speed>=1m/s 统计）

- `opp_ratio_6s=0.0`（7/7，未出现“相向追再掉头”）。
- `dt_to_carrier_speed>=1m/s`：均值 `4.00s`，最大 `4.14s`。
- 首次 guard 方向（`first_guard_dir_horizon`）随轨迹变化明显：
  - 基线样本：`(0.53,-0.85)@h=6.18`
  - 变轨迹 A：`(-0.75,0.67)@h=7.79`
  - 变轨迹 B：`(-0.71,-0.71)@h=5.78`

## 产物

- 汇总指标：`report/20260411_predictive_generic_direction_batch_metrics.csv`
- 单组 GIF 可视化（每个 run）：`results/<RUN_ID>/trajectory_xy_full.gif`
