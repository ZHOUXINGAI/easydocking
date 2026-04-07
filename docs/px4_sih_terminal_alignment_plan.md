# PX4 SIH Docking：终端对齐（横向 0.2m + 纵向安全带）Long-Term Plan

本文件是 **长期执行清单**（M0–M3）。后续每次改进都必须：

1. 先对照本文件确认当前要推进的里程碑/子目标；
2. 只做与该子目标直接相关的改动（避免跑偏）；
3. 跑一组可复现的结果（至少 1 次；建议 5–10 次小 batch）；
4. 用 `results/px4_sih_batch_summary.csv` + `docking_log.csv` 验收；
5. **完成一个子目标就把对应 checkbox 勾掉**（并附上 result 目录名作为证据）。

约束（来自项目当前共识）：
- 不调全局速度（除非有非常直接证据）。
- 不做 crude squeeze hacks（不靠放宽完成阈值“假成功”）。
- 继续使用 automated classification + batch summary 工作流。
- 聚焦 “good START but downstream fail”，尤其是被动 corridor retained / retry / terminal handoff。

---

## 最终验收定义（不要求进入 COMPLETED）

目标是 **终端队形对齐可持续保持**（而不是“贴上去”）：

- 横向：`abs(controller_terminal_lateral_error) <= 0.20 m`
- 纵向安全带：`rel_z = mini_z - carrier_z` 在 `(0.20 m, 1.00 m)` 内
- 持续保持：上述两条同时满足，连续时间 `>= 0.3 s`（建议后续提升到 `0.5 s`）

工程建议（留边界余量）：
- 统计/验收时用 `0.25 <= rel_z <= 0.95` 作为 z-band（避免卡边界抖动）。
- 统计/验收时增加 “终端距离门槛”：`relative_distance <= 10.0 m`（避免远距离下横向误差偶然很小造成假阳性）。

数据源：
- `controller_terminal_lateral_error` 来自 controller debug（CSV 列：`controller_terminal_lateral_error`）。
- `rel_z` 来自 docking status（CSV 列：`rel_z`，为 mini 相对 carrier 的 z）。

---

## 终极目标验收（FINAL_PASS / 真实对接）

目标是 **真实对接/附着语义成立**，不仅仅是“终端队形对齐”。只有同时满足下列条件，才算 FINAL_PASS：

- 平面误差：`sqrt(rel_x^2 + rel_y^2) <= 0.20 m`（或等价：`abs(along_error) <= 0.20` 且 `abs(lateral_error) <= 0.20`）
- 高度带：`rel_z = mini_z - carrier_z` 在 `0.20–1.00 m` 内（验收统计用 `0.25–0.95` 作为安全带）
- 末段相对速度几乎为 0：`|rel_v| <= 0.45 m/s`（对齐 prototype capture 口径，可按实验再收紧）
- 航向一致：`abs(yaw_mini - yaw_carrier) <= 20 deg`（如 yaw 不可信需单独说明）
- 连续保持：以上条件连续保持 `>= 0.3 s`（推荐最终提升到 `0.5 s`）
- 语义成立：必须进入 `COMPLETED`（或等价“刚性附着/完成”语义）

数据源：
- `rel_x/rel_y/rel_z/rel_v*` 来自 docking status / docking_log.csv
- `controller_terminal_along_error / controller_terminal_lateral_error` 来自 controller debug
- `mini_yaw_deg / carrier_yaw_deg` 来自 odom logger

---

## M0：把“0.2 lateral + z-band”变成可回归的批量指标

目的：让每次迭代都能在表格里量化“离 0.2 还有多远”，避免靠主观看图。

- [x] M0.1 在 `scripts/summarize_px4_sih_batch.py` 增加列：`min_terminal_lateral_error_m`
- [x] M0.2 增加列：`hold_lat_0p2_zband_sec`（`abs(lat)<=0.2 && 0.25<=rel_z<=0.95` 的最长连续保持时间）
- [x] M0.3 增加列：`hold_lat_0p35_zband_sec`（同上但 lateral 阈值 0.35，用于更早期里程碑）
- [x] M0.4 增加列：`docking_entry_count` 与 `second_docking_entry_lat_abs_m`（量化 retry 是否带来改善）

证据（已刷新 `results/px4_sih_batch_summary.csv`）：
- 表头已包含：`min_terminal_lateral_error_m`, `hold_lat_0p35_zband_sec`, `hold_lat_0p2_zband_sec`, `docking_entry_count`, `second_docking_entry_lat_abs_m`
- 示例：`20260329_172329_px4_sih` 在 z-band 内 `hold_lat_0p2_zband_sec ≈ 13.04s`（对应 `rel_z≈0.495m`，`abs(lat)` 最小值约 `0.0005m`）

验收方式：
- 更新后的 `results/px4_sih_batch_summary.csv` 能直接 `filter/sort` 出：
  - `min_terminal_lateral_error_m` 最小的 runs
  - `hold_lat_0p2_zband_sec` 非 0 的 runs（目标是从 0 开始出现）

---

## M1：2.6m → 0.8m：让 TRACKING 成为横向收敛器

背景现象（当前瓶颈）：
- good START 后能进 DOCKING，但 `controller_terminal_lateral_error` 常卡在 `~2.4–2.7m`，无法进入 0.2m 级别。

策略原则：
- **TRACKING 用于收敛横向误差（追 corridor point）**
- **DOCKING 不负责从 2m 收到 0.2m**；DOCKING 更像“检测是否进入可终端域，否则触发 retry”。

- [x] M1.1 `TRACKING -> DOCKING` gate：当 `abs(terminal_lateral_error)` 仍然较大时禁止进入 DOCKING（门槛以数据调参）
- [x] M1.2 batch 分布上 `min_terminal_lateral_error_m` 进入 `< 0.8m` 的样本比例显著上升（写明 batch 目录作为证据）

验收阈值（阶段性）：
- `min_terminal_lateral_error_m < 0.8`
- `hold_lat_0p35_zband_sec` 开始从 0 变成非 0（即进入“可终端对齐区”）

---

## M2：0.8m → 0.35m：让 retry 真正改善横向（避免相位抖动）

目标：出现 `DOCKING -> TRACKING -> DOCKING` 时，第二次拦截必须显著更好，而不是“立刻回 DOCKING”。

- [ ] M2.1 观察到 `DOCKING -> TRACKING -> DOCKING`（至少 1 个 run；记录 result 目录）
- [ ] M2.2 `second_docking_entry_lat_m` 明显小于 `first_docking_entry_lat_m`（建议改善 ≥30%）
- [ ] M2.3 batch 分布上 `min_terminal_lateral_error_m < 0.35` 开始稳定出现（写明 batch 目录）

常见失败模式与对策（仅限本里程碑范围）：
- retry 后 TRACKING 停留太短：增加 settle / 进入 DOCKING 的进度 gate（例如 along 不可太离谱）。
- retry 触发条件太晚：让“lateral 不收敛 + corridor retained”能更早触发 retry，但不要动全局速度。

---

## M3：0.35m → 0.20m：终端语义对齐 z 安全带 + 横向 0.2m hold

目标：不是贴合捕获，而是把终端队形稳定到“横向 0.2m + z-band”并保持。

- [ ] M3.1 所有终端相关窗口/候选条件显式约束 `rel_z` 在安全带内（建议统计用 `0.25–0.95`）
- [ ] M3.2 控制律的 z 目标不会把 mini 拉到 `<0.25m`（避免“贴着”）
- [ ] M3.3 `hold_lat_0p2_zband_sec >= 0.3s` 在 batch 中出现并可复现（写明 batch 目录）
- [ ] M3.4 将保持时间目标从 `0.3s` 提升到 `0.5s`（可选，但推荐最终做到）

---

## M4：从“对齐保持”到“速度匹配 + 捕获窗口”

目标：把末段相对速度压到 **真实可捕获窗口**，避免“距离小但速度还大”的假成功。

- [ ] M4.1 在 `results/px4_sih_batch_summary.csv` 增加 `final_pass` 相关列（平面误差 / rel_speed / yaw / hold_sec / completed）
- [ ] M4.2 DOCKING 末段显式优先速度匹配（along/lateral/vertical），并把捕获判据与 `rel_speed` 绑定
- [ ] M4.3 批量中出现 `best_window_rel_speed_mps <= 0.45` 且保持时间 `>=0.3s` 的样本（写明 batch 目录）
- [ ] M4.4 `COMPLETED` 不是偶发：满足 FINAL_PASS 的样本可复现（至少 5-run 中出现）

---

## M5：COMPLETED 语义稳定化（真实“贴合/附着”）

目标：让 `COMPLETED` 进入后保持稳定，不是“单帧擦过”。

- [ ] M5.1 `COMPLETED` 进入条件与 FINAL_PASS 严格对齐（避免假完成）
- [ ] M5.2 `post_completed` 保持窗口稳定（>=0.5s，速度/姿态无突变）
- [ ] M5.3 批量验证达到最终统计门槛（例如 10 次≥9 次 FINAL_PASS）

---

## 迭代记录（每次改动后追加）

格式建议：

```text
YYYY-MM-DD
- change: <一句话描述改动>
- runs: <result_dir_1>, <result_dir_2>, ...
- batch: <batch_dir 如果有>
- metrics: min_lat=..., hold_0p2_zband=..., second_docking_lat=...
- checkbox: 勾掉了哪些子目标
```

2026-03-30
- change: M0: `px4_sih_batch_summary.csv` 新增终端对齐指标列（min lateral / hold time / docking entry）
- runs: 通过全量刷新 `results/px4_sih_batch_summary.csv` 验证列存在且可筛选
- metrics: `20260329_172329_px4_sih` 在终端距离门槛内 `hold_lat_0p2_zband_sec ≈ 13.04s`
- checkbox: 勾掉 M0.1–M0.4

2026-03-30
- change: M0: 终端统计口径补充 `relative_distance <= 8.0m`（避免远距离假阳性）
- runs: 已刷新 `results/px4_sih_batch_summary.csv`，`hold_lat_*` 与 `min_terminal_lateral_error_m` 统一受该距离门槛约束
- metrics: `20260329_152648_px4_sih` 的远距离假阳性 hold 已被过滤为 0

2026-03-30
- change: M1.1: 收紧被动 `TRACKING -> DOCKING` 进入门槛（新增 `abs(terminal_lateral_error) < 0.90` gate）
- runs: `20260330_012225_px4_sih`, `20260330_013202_px4_sih`
- metrics: 两个 run 均 `docking_entry_count=0`（gate 生效），且 `final_phase=TRACKING`（避免带着 ~2.7m lateral 直接进 DOCKING）
- checkbox: 勾掉 M1.1；M1.2 效果待 batch 验证

2026-03-30
- change: M1.2: 强化横向收敛（主要在 DOCKING 的非 corridor 分支放开横向速度上限；并调整被动 APPROACH 的进入逻辑，使其在接近终端窗口时可直接进入 DOCKING）
- runs: `20260330_123210_px4_sih`, `20260330_123837_px4_sih`, `20260330_124055_px4_sih`, `20260330_124730_px4_sih`, `20260330_125405_px4_sih`
- metrics: `scripts/validate_m1_2_tracking_lateral_batch5.py` 连续 5 个有效样本全部 PASS（`min_lat < 0.8m`，eval zone: `relative_distance<=10m`, `rel_z in [0.2,1.2]`)
- checkbox: 勾掉 M1.2
