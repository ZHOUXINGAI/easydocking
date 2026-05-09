# EASYDOCKING — Design & Verification Checklist

**每次改动之前先扫一遍，每次跑完测试逐条对照。**

---

## A. 设计硬约束（改动前必须满足）

- [ ] **A1. 两架飞机都规划轨迹**：CorridorPlan 必须同时给 carrier 和 mini 规划轨迹，各飞各的。不能只规划一架。
- [ ] **A2. Carrier 轨迹**：预规划的弧线/直线，切线连接 mini 轨道圆。不追 mini 当前位置。
- [ ] **A3. Mini 轨迹**：沿轨道到触发相位 → 切出沿切线直飞。需定义切出逻辑。
- [ ] **A4. 规则驱动**：所有参数（切点、时长、速度）由几何/物理规则计算，不靠猜参数。
- [ ] **A5. CorridorPlan 发布**：轨迹固定后才发布到 `/docking/corridor_plan`，不是每周期更新。
- [ ] **A6. 同步到达**：两机在 rendezvous 点距离 ≤ 5m。
- [ ] **A7. 终端对接**：距离 < 5m 后切换到精准对接模式。

## B. 执行硬约束（必须在结果中体现）

- [ ] **B1. Front-consistency**：carrier 从开始运动到对接完成全程在 mini 前面。做不到就是失败。
- [ ] **B2. Carrier 不等 mini**：carrier 出发后不能蹲在 rendezvous 点等 mini，也不能半路停下来等。释放时机由计划决定，一旦出发就持续移动。
- [ ] **B3. 轨迹形状**：carrier 路径近乎弧线，末端是轨道圆的切线。mini 路径是圆弧 + 直线。
- [ ] **B4. Hold=0**：无 departure hold。carrier 从 START 后立即开始按计划移动。
- [ ] **B5. 不需要 reverse**：carrier 的速度方向不出现 >90° 的反转。

## C. 输出要求

- [ ] **C1. trajectory_xy_full.png** 上必须画出 CorridorPlan 轨迹（虚线），包含两机各自规划路径。
- [ ] **C2. x_time.png** 上必须画出计划轨迹线（虚线）和发布时刻标记。
- [ ] **C3. GIF 加速**：1 真实秒 = 多模拟秒（≥15x），interval/fps 配置正确。

## D. 每轮验证流程

1. 跑 `bash scripts/run_mock_docking.sh`
2. 读 `summary.txt` 检查 FINAL_PASS、路径长度、相位序列
3. 看三张图：trajectory_xy_full.png、x_time.png、speed_profile.png
4. 对照 A/B/C 清单逐条打勾，未满足的标红
5. 把结果汇报给用户，红了的先修
