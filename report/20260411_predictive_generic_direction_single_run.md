# Predictive抢位泛化验证（单组）

- 时间：2026-04-11
- Run：`results/20260411_182737_px4_sih`
- 配置：`START_RVIZ=true AUTO_START_REAR_ENTRY_PREDICTION_PRIMARY_GATE=true`

## 本次改动

- `px4_offboard_bridge.py` 的同向守卫从“固定时域线性预估”升级为：
  - 自适应预测时域（随相对距离/速度变化）
  - 可选加速度外推（非固定方向）
  - 反向保护（预测方向若与目标未来切向相反则回退到未来切向）
  - force阶段最小前推速度（`same_direction_guard_force_min_forward_command_mps`）

## 关键结果

- `classification=final-pass`
- `first_non_idle_t=77.340s`
- `t(carrier_speed>=1m/s)=81.241s`
- `dt_to_speed>=1m/s=3.901s`
- 方向一致性（仅统计 `carrier_speed>=1m/s`）：
  - 3s窗口：样本不足（起步阶段速度仍较低）
  - 6s窗口：`opposite_ratio=0.000`，`dot_mean=0.653`

## 观察

- 抢位方向不依赖全局方位（不是写死“东南”）；由 mini 的未来轨迹与相对几何在线决定。
- 早期“相向飞”现象已显著抑制（有效速度样本下无反向）。
- 仍有起步加速滞后（约 3.9s），后续可继续从固定翼响应链路（速度/姿态/能量）再压缩。
