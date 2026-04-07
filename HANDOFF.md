# Handoff: easydocking M3 (2026-03-31)

## Goal
Achieve **M3 5/5 PASS** with:
- `hold_lat_0p2_zband_sec >= 0.30s`
- `abs(lat) <= 0.2`
- `rel_z` in `[0.25, 0.95]`
- `relative_distance <= 10m`

## Status Snapshot (What we did / What we will do / Final target)
**Done so far**
- Implemented **Plan A strong convergence** in `src/easydocking_control/src/docking_controller.cpp`
  - Added `tracking_lat_strong` and strengthened lateral interception
  - Increased lateral velocity limits when `|lat|>1.0 && dist<12`
  - Suppressed along speed in strong-lat regime to avoid drifting beyond 10m
  - Faster pull-in to “directly under mini”
- Implemented **fine-band damping** (tracking_lat_fine, lower lateral limits, higher damping)
- Hold no longer hard-freezes along velocity (small closure allowed)
- **Plan B partial (z-band + frame alignment)**:
  - Increased controller `desired_relative_position.z` in `src/easydocking_control/launch/docking.launch.py` from **0.4 -> 0.6** to keep `rel_z` away from the lower band edge (0.25) and make hold time achievable.
  - Aligned TRACKING/DOCKING control tangent-frame (`track_direction`) with the same target-velocity LOS convention used by `transitionPhase()` (controller debug / metrics), reducing “lat drift” caused by frame mismatch.
  - Adjusted DOCKING “lat-hold” to clamp **relative-to-target** along/lateral deltas (instead of absolute speeds) so the carrier can still follow mini feedforward while keeping terminal lateral error small.

**Next step (short-term)**
1. Run a 5-run batch with current A changes.
2. Check whether **min_lat consistently < 0.5** and more runs enter `|lat|<=0.2`.
3. If A improves entry, move to **Plan B**: extend hold time to >=0.3s by refining fine-band damping and distance control.

**Final target (long-term)**
- Two-layer acceptance: `M3_PASS` + `FINAL_PASS`
- Final docking requires COMPLETED + tight terminal error + low relative speed + sustained hold.

## Long-Term Plan (Final Docking Success)
We must eventually switch from **M3 semantic pass** to **Final Docking pass**:

**Final success (target):**
- `abs(terminal_lateral_error) <= 0.20m`
- `rel_z` in `(0.20, 1.00)` (mini above carrier, not too high)
- Relative speed ~0 (low relative speed and aligned heading)
- **Sustained** hold (>=0.3s) in band
- Enter **COMPLETED** / attached semantic state

**Implementation plan (long-term):**
1. **Two-layer reporting** (avoid “pass but not actually docked”):
   - `M3_PASS`: current hold-based rule
   - `FINAL_PASS`: requires COMPLETED + terminal errors + speed/heading constraints
2. **Control-side dynamics constraints**:
   - Carrier speed cap: ~**12 m/s**
   - Carrier acceleration cap: **2.5 m/s^2**
   - Mini uses true airspeed (TAS) for speed gating (avoid plot/estimation artifacts)
3. **Start-window gating**:
   - Keep geometry cluster gate default **false** (avoid random timeouts)
   - Treat start-window-fail as invalid run and re-run
4. **Evaluation**:
   - M3 batch: 5/5 PASS
   - Final batch: 5/5 FINAL_PASS

## Current Plan
**A -> B**
- **A (Strong convergence):** fix runs that never reach `|lat| <= 0.2` (min_lat stays > 1.0).
- **B (Fine-band stability):** once A is solved, stabilize hold time to >=0.3s.

## Key Code Changes (Plan A already applied)
File: `src/easydocking_control/src/docking_controller.cpp`

Added/adjusted in tracking:
- `tracking_lat_strong` when `|lat| > 1.0 && dist < 12`
  - Higher lateral intercept gain/limit
  - Higher lateral velocity limits
  - Stronger lateral P/D gains
  - Stronger along-speed suppression (avoid drifting beyond 10m)
  - Faster pull-in to “directly under mini”
- `tracking_lat_fine` when `|lat| < 0.30 && dist < 10 && z in band`
  - Lower lateral velocity limit
  - More damping, less P gain
  - Smaller hold lateral limit
- Hold no longer hard-freezes along velocity (small along closure allowed to avoid distance >10m).

## Known Pitfalls / Historical Issues
- “PASS but not docked”: M3 pass is semantic; final docking not guaranteed.
- Speed plot spikes caused by dt=0 samples (fixed earlier in report generator).
- Old logs used differential speed; prefer PX4 true airspeed for mini.

## Required UX Behavior
User wants to **see ongoing work** (continuous output, not “idle”).  
When running batches, keep streaming output and provide periodic status updates.

## How to Check/Stop Running Jobs
Check running:
```
pgrep -af run_px4_sih_docking_experiment.sh
pgrep -af px4
```
Stop all:
```
pkill -f run_px4_sih_docking_experiment.sh || true
pkill -f /home/hw/PX4-Autopilot/build/px4_sitl_default/bin/px4 || true
pkill -f "ros2 launch easydocking_control" || true
pkill -f summarize_px4_sih_batch.py || true
```

## How to Run a 5-Run Batch (visible progress)
```
for i in 1 2 3 4 5; do
  echo "=== RUN $i/5 ==="
  ./scripts/run_px4_sih_docking_experiment.sh
done
```

## How to Summarize Latest 5 Runs
Get latest 5 run directories:
```
ls -1dt results/*_px4_sih | head -5
```
Then summarize:
```
python3 - <<'PY'
import csv, sys
runs = [line.strip().split('/')[-1] for line in sys.stdin if line.strip()]
rows={}
with open('results/px4_sih_batch_summary.csv', newline='') as f:
    reader=csv.DictReader(f)
    for r in reader:
        if r['run'] in runs:
            rows[r['run']] = r
print('found', len(rows))
for rid in runs:
    r = rows.get(rid)
    if not r:
        print(rid, 'MISSING'); continue
    print(rid,
          'hold', r.get('hold_lat_0p2_zband_sec'),
          'entries', r.get('docking_entry_count'),
          'min_lat', r.get('min_terminal_lateral_error_m'),
          'best_lat', r.get('best_lateral_m'))
PY
```

## Next Actions
1. Run A batch, check if min_lat consistently < 0.5.
2. If A succeeds, proceed to B: tighten fine-band damping to extend hold >=0.3s.

## Latest Batch Notes (2026-03-31)
After applying the Plan B changes above, a 5-run batch still shows **inconsistency**:
- Some runs reach `hold_lat_0p2_zband_sec` well above 0.30s.
- Some runs still never reach `abs(lat)<=0.2` within 10m (hold=0), meaning lateral convergence remains the remaining blocker for true **5/5 PASS**.

Example recent 5-run batch IDs (20260331_212026 / 212252 / 212522 / 212754 / 213024):
- 3/5 achieved `hold_lat_0p2_zband_sec >= 0.30s`
- 1/5 had `hold_lat_0p2_zband_sec = 0.14s` (lat+z band achieved briefly but not sustained)
- 1/5 failed to reach `abs(lat)<=0.2` (min_lat stayed > 0.2)

## Immediate Next Work
1. Focus on the remaining failure mode: runs that never reach `abs(lat)<=0.2` within 10m.
2. Add a targeted fix to improve worst-case lateral convergence (without breaking the now-good hold stability), then rerun 5-run validation.

---

# Update: 2026-04-06 (next iteration notes)

## 1) 现在干了什么（已改动点）
- 继续按 `HANDOFF.md` 的 A->B 思路推进，并对 `src/easydocking_control/src/docking_controller.cpp` 做了几轮针对性调整（均已 `colcon build --packages-select easydocking_control` 验证可编译）：
  - TRACKING：把 close-range blend / lateral priority 的触发区间前移（希望更早把横向误差压下去，避免进入 <=10m 时仍大偏差）。
  - TRACKING：扩大 z-band “软边界”保护（close-range 下避免 vertical_cmd 把 rel_z 推到 0.25 边界外，导致 hold 计时被反复清零）。
  - APPROACH：增加 z-out-of-band 时的水平闭合抑制（希望在 z 大偏差时不要过早贴近到 <=10m）。
  - TRACKING：修正 `tracking_lat_hold_active` 下沿向速度上限的缩放（之前是裸 0.35/0.45，现按 `carrier_tracking_speed_limit_` 比例缩放，避免沿向几乎“停住”导致距离快速跑飞）。
  - TRACKING：尝试移除过强的沿向硬限速，并加了“给横向留额度”的沿向上限（通过 `sqrt(vmax^2 - v_lat^2)`，避免最终 norm clamp 把横向也缩小）。

## 2) 目前遇到的问题
- **结果仍不稳定**：同一套参数下，有些 run 能达到 `hold_lat_0p2_zband_sec >= 0.30`，但仍会出现“进入 <=10m 时 rel_z 仍远超 0.95 / 横向误差仍大”的 run，导致 hold=0。
- 一个典型失败模式：在 `APPROACH` 阶段 **z 还没收敛到 band**（例如 rel_z≈7m）时，3D 距离已经进入 <=10m；等 z 真正进 band 时，横向几何已经坏掉/或距离已经跑到 >10m。
- 另一个失败模式：目标沿向速度略高于 `carrier_tracking_speed_limit` 时，carrier 在 TRACKING 中可能长期“追不上”，导致 along error 变大，难以在 <=10m 内维持。

## 3) 下一步准备做什么
- 先把“会明显影响结果的实验参数”作为一个 A/B 开关验证：
  - 用 `CARRIER_TRACKING_SPEED_LIMIT=12.0` 运行 5-run batch（环境变量覆盖，无需改脚本）观察是否能显著减少“追不上导致距离跑飞”的失败。
- 若仍存在“z 未进 band 就贴近 <=10m”的失败：
  - 在 `APPROACH` 增加更强的 **z-before-close standoff 逻辑**（当 `rel_z` 超过上边界较多时，限制/反向水平闭合，确保 3D distance 不会先落入 <=10m）。
- 若希望更快拿到 5/5：评估是否允许启用 `wait_for_docking_window.py` 的 `enable_geometry_cluster_gate`（把明显“坏起点”判为 invalid 并自动重抽窗口）。

## 4) 最终要达成什么目标
- 仍以 **M3 5/5 PASS** 为第一目标：
  - `hold_lat_0p2_zband_sec >= 0.30s`
  - `abs(lat) <= 0.2`
  - `rel_z` in `[0.25, 0.95]`
  - `relative_distance <= 10m`
- 达标后再推进 long-term “Final Docking pass”（COMPLETED + 终端误差/速度/航向约束）。

## Evidence / Quick numbers (2026-04-06)
- 5-run batch（未额外设置 env 的那一轮）：
  - `20260406_212019_px4_sih`: hold≈0.301, min_lat≈0.089
  - `20260406_212249_px4_sih`: hold=0.0,   min_lat≈0.018（主要被 z-band 掉线影响）
  - `20260406_212516_px4_sih`: hold≈0.460, min_lat≈0.042
  - `20260406_212742_px4_sih`: hold≈0.400, min_lat≈0.002
  - `20260406_213008_px4_sih`: hold=0.0,   min_lat≈3.287（仍存在“横向永远进不了 0.2”的坏例）
- SPEEDSMOKE（仅验证：`CARRIER_TRACKING_SPEED_LIMIT=12.0`）：
  - `20260406_221058_px4_sih`: hold≈0.340, min_lat≈0.123（达标样本）
  - `20260406_221328_px4_sih`: hold=0.0,   min_lat≈2.450（仍失败，主要发生在 z 未收敛前进入 <=10m）

---

# Update: 2026-04-07（继续解决 2) 遇到的问题）

## 2) 目前遇到的问题（最新定位）
复盘近期失败（例如 `20260407_004839_px4_sih` / `20260407_005335_px4_sih`）后确认：
- “第一次进入 <=10m 时 rel_z 仍很大（~9m）”并不是坏例专属，**好例也会这样**；真正导致 `hold_lat_0p2_zband_sec=0` 的坏例，是在 <=10m 内 **z-band 与低横向误差窗口重叠太少**：
  - 有的 run 在 <=10m 时 z-band 占比只有 ~4–5%，很快从 `rel_z>0.95` 直接冲到 `rel_z<0.25`（几乎不给 hold 计时窗口）。
  - 即使 `min(best_lateral_m)` 能短暂变小，往往发生在 `rel_z<0.25`（出 band）而不是在 band 内。

## 已做的改动（针对问题#2）
文件：`src/easydocking_control/src/docking_controller.cpp`
- `approachPhaseControl()`：
  - 增加 **position-setpoint 的横向 pull-in**：横向误差大且距离近时，从 lead-point 逐步切到“直接追 mini 正下方”（提高 worst-case 横向收敛）。
  - 增加 **close-range 垂直速度限幅**：在 `relative_distance<15m` 且 `rel_z` 接近 band 时，限制 `carrier_velocity_command_.z()`，避免“快速穿过 z-band”导致 hold 窗口太短。
- `trackingPhaseControl()`：
  - 调整 close-range 的 along 轴减速逻辑：当 `along_error` 仍显著为正（carrier 落后）时，不要过度压 along（避免长期追不上导致几何/横向停滞）。
- `dockingPhaseControl()`：
  - 增加 z-band guard：在 close-range 时避免继续把 `rel_z` 往 band 外推（防止 DOCKING 期间的垂直 overshoot 让 band 掉线）。

文件：`src/easydocking_control/launch/docking.launch.py`
- 恢复 carrier 侧 `use_velocity_feedforward=True`（让 controller 输出的速度 setpoint 能进入 PX4 offboard 链路）。

## 证据（单次 smoke 的显著改善）
`20260407_010344_px4_sih`（`CARRIER_TRACKING_SPEED_LIMIT=13.0`）：
- <=10m 内 `rel_z` 落在 `[0.25, 0.95]` 的占比约 **38%**（此前坏例常见 ~4–5%）。
- z-band 内 `min |lat|` 可到 **~0.053m**（横向已能压到 0.2 内）。
- 但 `hold_lat_0p2_zband_sec` 仍只有 **~0.14s**（离 0.30s 还有差距），说明瓶颈已从“几何无法进入”转为“近距离稳定保持不足”。

## 下一步（从 A 转入 B：补 hold 稳定）
1. 先把 `hold_lat_0p2_zband_sec` 稳定拉到 >=0.30s（优先看 DOCKING/close-range 的 lateral/z 抑振与 release 条件）。
2. 在单次 smoke 能稳定 >=0.30s 后，再跑 5-run batch 做 5/5 验证。

---

# Update: 2026-04-07（继续推进：定位 “lat 已变小但 <=10m 掉线” + 修复批量汇总卡住）

## 1) 现在干了什么（已改动点）
### A. 运行脚本/汇总性能（避免“跑完卡住没输出”）
- `scripts/summarize_px4_sih_batch.py`：新增参数
  - `--latest N`：只汇总最新 N 个 `results/*_px4_sih`（0 表示全量）
  - `--pattern`：自定义 glob（默认 `results/*_px4_sih`）
- `scripts/run_px4_sih_docking_experiment.sh`：
  - 增加可见输出：打印 `RESULTS_DIR`、`CARRIER_TRACKING_SPEED_LIMIT`、以及 post 阶段进度（generate_report / classify / summarize）
  - 调用汇总脚本改为：`summarize_px4_sih_batch.py --latest "$BATCH_SUMMARY_LATEST_N"`（默认 120），避免全量扫 800+ runs 导致卡很久

### B. 控制器：让 B 阶段 hold 更容易“触发并持续”（重点解决 <=10m 掉线）
文件：`src/easydocking_control/src/docking_controller.cpp`
- 提前进入 “lat-hold latch”：
  - `terminal_lat_hold_condition` / `tracking_lat_hold_condition`：从 `abs(lat)<0.20` 放宽到 `abs(lat)<0.24`（先 latch 再把 lat 压进 0.20 内，减少 overshoot）
- DOCKING safety-band 的近距离横向抑振更早介入：
  - `safety_band_hold_blend` 的距离因子从 `(7.0 - terminal_distance)/3.0` 改为 `(10.0 - terminal_distance)/4.0`
- 强化 A 阶段横向收敛（提高进入 B 的概率）：
  - APPROACH：position-setpoint pull-in 的横向 blend 更早触发（阈值从 1.0m 降到 0.6m）
  - TRACKING：`tracking_lat_strong` 与 `tracking_lateral_priority` 的触发范围前移（更早开始“转向压横向”）
- 关键新定位对应的控制补丁（<=10m 贴边掉线）：
  - TRACKING lat-hold 时：根据 `distance≈10m` 和 `along_relative_speed>0` 动态增大沿向 delta 限幅，并加一个小的“距离守门”沿向速度 bias，尝试把 `relative_distance` 拉回到 <=10m 内，让 `hold_lat_0p2_zband_sec` 能累计起来

## 2) 目前遇到的问题（最新定位）
- 已观察到一种关键失败：**lat 在 z-band 内其实能压到很小，但恰好发生在 `relative_distance` 刚好 > 10m 的时刻**，导致 `hold_lat_0p2_zband_sec` 仍然为 0。
- 这说明 B 阶段除了 DOCKING 横向/垂向抑振外，还必须处理：
  - 在 `abs(lat)` 接近阈值时，如何 **不让距离贴着 10m 边界外漂**（尤其是 mini 沿向把 carrier “拉走”时）。

## 3) 下一步准备做什么
1. 用当前版本继续跑 5-run batch（建议先 smoke 2~3 次确认没有明显退化），关注：
   - 是否出现 `min_lat_band_close < 0.20` 的 run
   - 一旦出现，`hold_lat_0p2_zband_sec` 是否显著提升（目标 >=0.30s）
2. 若仍频繁出现 “lat 很小但 d>10”：
   - 进一步增强 TRACKING/DOCKING 的 “<=10m 距离守门”逻辑（沿向/总速度限制的微调）
   - 或评估把 `CARRIER_TRACKING_SPEED_LIMIT` 从 13.0 再上探（若 mini along speed 仍偏高）

## 4) 最终要达成什么目标
- 仍以 **M3 5/5 PASS** 为第一目标：
  - `hold_lat_0p2_zband_sec >= 0.30s`
  - `abs(lat) <= 0.2`
  - `rel_z` in `[0.25, 0.95]`
  - `relative_distance <= 10m`

## Evidence（用于复盘 <=10m 掉线）
- `20260407_031409_px4_sih`：在 z-band 内 `lat` 会从 `~0.26 -> 0.21 -> 0.18 -> 0.09 -> 0.03` 快速变好，但对应 `relative_distance` 同步从 `~9.84 -> 10.00 -> 10.11 -> 10.34 -> 10.28`（刚越过 10m），因此 `hold_lat_0p2_zband_sec` 仍为 0。

---

# Update: 2026-04-07（继续：新增两个“关键卡点” + DOCKING 横向 PD 扩展）

## 新观察（更具体的失败机制）
1) **lat 已进入 0.2，但连续段不够长**  
例如 `20260407_110529_px4_sih`：在 `d<=10m & z∈[0.25,0.95]` 内 `min |lat|≈0.158m`，但基本是“单点/短段”，`hold_lat_0p2_zband_sec` 仍为 0。

2) **lat 满足时，distance 很快漂出 10m**  
例如 `20260407_102251_px4_sih`：`hold_lat_0p2_zband_sec≈0.14s`，之后并不是 lat/z 先坏，而是 `relative_distance` 从 `~9.93m` 漂到 `>10m`（如 `10.09m`），hold 计时被清零。

## 本轮新增改动（摘要）
文件：`src/easydocking_control/src/docking_controller.cpp`
- TRACKING：将 close-range 的速度 smoothing 降低（`0.04 -> 0.02`）以减少在小误差附近的“反向拉回”导致的横向回弹。
- TRACKING lat-hold：沿向速度 clamp 改为 **不允许显著慢于 mini**（below 侧限幅固定 0.20m/s），配合现有 near-window bias，减少 “<=10m 边界外漂”。
- DOCKING：把“terminal-frame 横向 PD”从仅 latch(<0.2) 扩展到非 corridor 的常态 DOCKING 横向控制（gain/limit 随 |lat| 调度），提高 run-to-run 收敛一致性。

文件：`scripts/run_px4_sih_docking_experiment.sh` / `scripts/summarize_px4_sih_batch.py`
- batch 汇总默认只扫最新 N 个（避免 800+ runs 全扫卡住），并在 post 阶段打印可见进度。

## 近期结果快照（用于对比）
- 5-run batch（`20260407_103711~104451`）：仍 0/5（多数 run `min_lat` 停在 0.4~1.0+）。
- 单次出现“接近达标”的样本：
  - `20260407_102251_px4_sih`：`hold_lat_0p2_zband_sec≈0.14s`，随后因 `distance>10m` 掉线。
  - `20260407_110529_px4_sih`：`min |lat|≈0.158m`（在 band&<=10 内），但连续段过短（hold0.2 仍为 0）。

---

# Update: 2026-04-07（继续：解释“full path 没对接上 / xy 很大” + 沿向闭合迭代）

## 1) 现在干了什么（已改动点）
文件：`src/easydocking_control/src/docking_controller.cpp`
- **TRACKING/lat-hold latch 增加 along gate**：`terminal_lat_hold_condition` / `tracking_lat_hold_condition` 增加 `abs(terminal_along_error) < 3.2`，避免在 along 仍很大时进入 hold 相关逻辑导致“追不上、distance 漂出 10m”。
- **TRACKING：沿向闭合补强（针对 d≈10m 边界外漂）**
  - 用 `terminal_along_error`（而不是 tracking error 的 along）做 catch-up gating，避免“lat 看起来很好但 along 仍很大”的错配。
  - close-range 增加更强的 along-speed bias（目标是让 carrier 在 `<=10m` 附近不要持续慢于 mini）。
  - 增加 **along floor + lateral budget 预留**：在 close-range 且 `terminal_along_error` 很大时，优先保证沿向不低于 mini 的 along 速度（考虑到 `carrier_max_accel=2.5` 的加速度限幅，否则来不及在 10m 内补回来）。
- **TRACKING：允许更早启动 catch-up**：把 close-range catch-up 从“仅 12m 内”提前到“18m 内开始 ramp”，给加速度限幅留出爬升时间。
- **TRACKING：沿向减速 gate 改用 terminal along**：只有当 `|terminal_along_error|` 已经足够小才允许明显减速，避免长期落后。

文件：同上（phase transition）
- **DOCKING 进入门槛放宽**：`(tracking_horizontal_distance > 10 || abs(terminal_lateral_error) < 1.0)` 放宽到 `< 1.6`，让 DOCKING 能更早接管并压横向（避免一直在 TRACKING 里兜圈后又掉出 10m）。

文件：同上（APPROACH）
- 增加 **z-before-close standoff**：当 `rel_z` 远超 z-band 上边界时，限制/反向水平闭合速度，避免“z 还没收敛就先贴到 <=10m”导致 hold 窗口极短（该项目前效果不稳定，仍在评估）。

## 2) 目前遇到的问题（最新定位）
- 用户看到的“full path 没对接上 / xy 仍大”，核心机制通常是：
  - 在 `d<=10m` 附近 mini 的水平速度仍较高（常见 ~12m/s），而 carrier 因 **加速度限幅 `carrier_max_accel=2.5`** + 横向占用预算/转弯掉速，导致沿向持续落后（`controller_tracking_along_speed` 长时间为正），从而 `relative_distance` 很快再次 >10m。
- 仍有明显 run-to-run variance：部分 runs `min_lat` 仍停在 0.8~1.0+（A 阶段横向未收敛），导致 hold=0。

## 3) 下一步准备做什么
1. 继续以 `CARRIER_TRACKING_SPEED_LIMIT=13.0` 做 5-run batch 验证是否能稳定把 `hold_lat_0p2_zband_sec` 推到 >=0.30s。
2. 若 “hold0.2 卡在 0.20~0.26”：
   - A/B 试验 `CARRIER_MAX_ACCEL=3.5`（只做验证：确认是否确实被加速度限幅卡住；若提升明显再讨论是否能保持 2.5 的物理约束）。
   - 或进一步前移/加大 TRACKING 的 catch-up（让 carrier 在进入 10m 前就已经接近 mini 速度）。

## 4) 最终要达成什么目标
- 仍以 **M3 5/5 PASS** 为第一目标：
  - `hold_lat_0p2_zband_sec >= 0.30s`
  - `abs(lat) <= 0.2`
  - `rel_z` in `[0.25, 0.95]`
  - `relative_distance <= 10m`

## Evidence（本轮结果，说明“沿向闭合”方向有效但仍未达标）
- `20260407_124807_px4_sih`：`hold_lat_0p2_zband_sec≈0.10s`，`min_lat≈0.092m`。
- `20260407_125709_px4_sih`：`hold_lat_0p2_zband_sec≈0.20s`，`hold_lat_0p35_zband_sec≈0.26s`，`min_lat≈0.003m`（已逼近 0.30s，但仍差一截）。

---

# Update: 2026-04-07（A/B 验证：是否被加速度限幅卡住 + 新增 DOCKING/TRACKING 稳定补丁）

## A/B 验证结论（重点：不是单纯 `carrier_max_accel`）
- 以 `CARRIER_TRACKING_SPEED_LIMIT=13.0` 跑批对比：
  - 默认 `CARRIER_MAX_ACCEL=2.5`：仍常见 `hold_lat_0p2_zband_sec` 卡在 **0.10~0.26s**，未见 >=0.30 的稳定样本。
  - `CARRIER_MAX_ACCEL=3.5`（验证用）：**最好的 hold0.2 仍是 ~0.26s**（例如 `20260407_133814_px4_sih`），并没有“直接跨过 0.30”的质变；说明瓶颈不只是在 accel limit。

## 关键复盘：`hold0.2≈0.26s` 的断点原因
- `20260407_133814_px4_sih`：最长连续段在 `t≈79.26~79.52`，满足 `d<=10m & z∈[0.25,0.95] & |lat|<=0.2`。
- 断点首先是 **`lat` overshoot**（`|lat|` 从 0.173 快速变到 0.242，越过 0.2），随后才是 `distance>10m`。
- 同时该段处于 **corridor DOCKING**（`controller_rendezvous_corridor_active=1`），提示 corridor 分支的 lateral 收敛/阻尼不足，会把 `terminal_lateral_error` 推过阈值从而重置 hold。

## 新增改动（尝试把 0.26s 推过 0.30s）
文件：`src/easydocking_control/src/docking_controller.cpp`
- **DOCKING corridor 分支增加 terminal-frame 横向 PD（仅在 metric window 附近生效）**
  - 条件：`terminal_distance<10 & z in band & |terminal_lateral_error|<0.35`
  - 作用：对 `terminal_lateral_error_now / terminal_lateral_speed_now` 做小幅 PD，抑制 overshoot，避免刚进 0.2 又被推出来。
- **TRACKING：沿向 smoothing 变为“沿向更强、横向更弱”**
  - close-range 下沿向匹配用更强 smoothing（额外 +0.05），减轻 accel limit 下的 along lag，目标是延缓/避免 `distance` 过快漂出 10m。

## 当前状态
- 仍未跑出 `hold_lat_0p2_zband_sec >= 0.30s` 的稳定 5/5（目前最好样本仍在 ~0.26s）。
- 后续建议：继续跑 smoke/小批直到出现 `hold0.2≈0.26s` 的 near-miss，再对比是否能被新 PD 抑制 overshoot 拉到 >=0.30s。

---

# Update: 2026-04-07（末端 XY 不对接：从“控制律”转为“PX4 跟踪能力/mini 速度”定位）

## 1) 现在干了什么（已改动点）
- `src/easydocking_control/src/docking_controller.cpp`
  - PASSIVE TRACKING/DOCKING corridor：把 corridor 的定义改为**每 tick 以 mini 当前位置重新 anchor**（`rendezvous_corridor_anchor_ = mini_position`），避免“anchor 固定但 axis/切向随目标速度变化”导致进度/沿向控制不一致。
  - TRACKING：`tracking_z_recovery_active` 下不再用“绝对沿向限速”，改为**围绕 mini along 速度的 delta 限制**，避免 z 恢复时沿向掉速导致距离跑飞。
  - TRACKING：补充“d>10 且几何还行时”的 along 追赶逻辑（仍受 PX4 实际跟踪能力影响）。
  - carrier accel limit：把速度变化限幅从“**向量范数限幅**”改成“**逐轴限幅**”，避免横向大修正时把沿向爬升一并压死（为后续进一步诊断做准备）。
- `src/easydocking_control/scripts/experiment_logger.py`
  - `docking_log.csv` 新增 `carrier_cmd_vx/vy/vz`，用于直接对比**控制器下发速度** vs **PX4 实际速度**。
- `src/easydocking_control/scripts/px4_offboard_bridge.py`
  - 新增参数 `use_position_setpoint`（默认 `true`，保持兼容）；曾尝试让 carrier 走 velocity-only offboard，但该配置在一次实验中出现明显退化，因此**launch 中未启用**（当前仍为 position+velocity feedforward）。
- `scripts/run_px4_sih_docking_experiment.sh` / `scripts/start_px4_sih_simulation.sh`
  - 默认 `MINI_TRACKING_SPEED_COMMAND` 从 `10.0` 调整到 `9.2`（但目前观察到 mini TAS 仍可能显著高于 speed SP，需要进一步处理 fixed-wing 速度跟踪/能量管理）。

## 2) 目前遇到的问题（最新定位）
- 用户看到的“trajectory_xy_full 末端没对接上 / XY 距离仍大”，在最新日志里已被**定性**：
  - 控制器在末端通常已经下发了更快的 carrier 速度，但 **PX4 实际 carrier 速度跟不上**。
  - 例：`20260407_155551_px4_sih`（末端 DOCKING）
    - `carrier_cmd_xy_speed≈12.00m/s`，但 `carrier_xy_speed≈9.68m/s`
    - `mini_px4_true_airspeed_mps≈10.75m/s`，且 `mini_tecs_true_airspeed_sp_mps≈8.46m/s`（mini 出现显著 overspeed）
  - 结论：末端 XY 拉不开的主因不再是“控制器没给 along 追赶”，而是“**mini 实际速度偏高 + carrier PX4 跟踪能力/限速/限加速度**”导致无法维持 `relative_distance<=10m`。
- 因此，仅继续加大 TRACKING/DOCKING 的 along bias 很可能**边际收益很低**（cmd 提高但 PX4 不跟）。

## 3) 下一步准备做什么
1. 把焦点从 `docking_controller.cpp` 转移到 **PX4 carrier 侧参数/模式**：
   - 尝试在 SITL 启动后对 carrier 设置更高的水平速度/加速度相关参数（例如 `MPC_*` 相关的 `VEL_MAX/ACC_MAX/JERK`），让实际速度能更贴近 `carrier_cmd_v*`。
   - 若无法稳定改 PX4 参数，则考虑：在评估脚本中以“carrier 实际可达速度”为约束，降低 mini 的实际速度（不是仅改 speed SP，而是解决 fixed-wing overspeed/能量策略导致 TAS 偏高）。
2. 针对 mini overspeed：
   - 排查 `px4_fixed_wing_bridge.py` 的终端速度选择与能量 guard，确认 speed SP 下发后 TAS 仍偏高的原因（爬升/下沉、航迹弯转、TECS 行为等）。
3. 做一次对照实验：保持控制器不变，仅调 PX4 carrier 参数或 mini 的能量/速度策略，观察末端 `relative_distance` 是否能稳定 <=10。

## 4) 最终要达成什么目标（不变）
- **M3 5/5 PASS**
  - `hold_lat_0p2_zband_sec >= 0.30s`
  - `abs(lat) <= 0.2`
  - `rel_z` in `[0.25, 0.95]`
  - `relative_distance <= 10m`

---

# Update: 2026-04-07（末端 XY 缩进：修复 mini 最低空速夹死 + carrier tilt 过高 failsafe）

## 1) 现在干了什么（已改动点）
### PX4 mini（fixed-wing）：解除最低空速硬夹
文件：`/home/hw/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/10041_sihsim_airplane`
- `FW_AIRSPD_MIN: 10.0 -> 8.5`
- `FW_AIRSPD_TRIM: 12.0 -> 11.0`
目的：让我们在 ROS 侧下发的 `mini_tracking_speed_command≈9.2`、`mini_docking_speed_command≈8.0` 不再被 PX4 固定翼的 `FW_AIRSPD_MIN` 直接夹回 >=10m/s，从源头降低末端追逐速度需求。

### PX4 carrier（quad）：提高可用倾角，但避免 SIH 姿态健康 failsafe
文件：`/home/hw/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/10040_sihsim_quadx`
- 新增/调整：
  - `MPC_TILTMAX_AIR=55`（原先尝试过 70，但会触发 `Attitude failure (roll)` 导致 failsafe/disarm）
  - `MPC_TILTMAX_LND=45`
目的：在末端需要追 mini 时给到更多水平加速度能力，同时避免 SIH 在极高 tilt 下触发姿态健康检查导致掉 OFFBOARD。

## 2) 目前遇到的问题（最新结论）
- 之前 `trajectory_xy_full.png` “末端没对接上”的根因之一，已确认是 **mini 实际空速被 `FW_AIRSPD_MIN` 夹死**：即使我们下发更低的 speed command，PX4 仍会强制维持 >=10m/s，导致 carrier 必须长期跑到 12~13m/s 才能保持 `d<=10m`。
- 盲目把 carrier `MPC_TILTMAX_AIR` 拉到 70 会在 SIH 中触发姿态健康检查（roll failure），直接导致 failsafe/disarm；因此 tilt 需要“适度放宽”，不能一味加大。

## 3) 证据（末端 XY 已显著缩进 / full path 已能对接）
- 新 smoke：`results/20260407_201541_px4_sih`
  - `final_distance_m=1.186`（`summary.txt`）
  - 末端 `rel_x≈1.107m, rel_y≈-0.226m`（见 `docking_log.csv` 最后一行）
  - mini 末端 TAS 明显下降：`mini_px4_true_airspeed_mps` 在最后 50 个样本均值约 `~10.57m/s`（旧跑法常见 ~11.2m/s+）
  - 对比用户提到的坏例：`results/20260407_143734_px4_sih` 末端 `relative_distance≈13.0m`、`rel_x≈12.68m`、`rel_y≈2.85m`
  - 末端轨迹图：`results/20260407_201541_px4_sih/trajectory_xy_full.png`

## 4) 下一步准备做什么
1. 再跑 2~3 次 smoke（不改控制器，仅验证 PX4 参数改动的稳定性），关注：
   - 是否还会出现末端 `relative_distance` 被拉回到 12~15m 的情况
   - mini 末端 TAS 是否能稳定压到 ~10.5m/s 左右
2. 如果“末端 XY 缩进”稳定后，把焦点切回 M3：跑 5-run batch，看 `hold_lat_0p2_zband_sec` 是否能稳定 >=0.30s。

---

# Update: 2026-04-07（末端 XY 再缩进 + M3 5/5 PASS）

## 1) 现在干了什么（已改动点）
- 控制器：`src/easydocking_control/src/docking_controller.cpp`
  - corridor DOCKING 的 **early-release for hold metrics** 从 `terminal_distance<3.0m` 放宽到 `terminal_distance<4.8m`，并增加 `|terminal_lateral_speed|` 约束（避免太抖时提前退出）。
  - 目的：让 DOCKING 更早退出 corridor 分支，给非-corridor 终端控制更多时间把 **末端 XY 继续收敛到接近 0**，同时减少 `|lat|` 刚进 0.2 又 overshoot 的概率。
- 实验脚本：`scripts/run_px4_sih_docking_experiment.sh`
  - 默认 `EXPERIMENT_DURATION_SEC: 85.0 -> 95.0`
  - 目的：避免 run 在末端仍在持续收敛时被“时间到”截断，导致 `trajectory_xy_full` 看起来“末端还没对接上”。

## 2) 目前遇到的问题
- 这一轮改动后，原先用户反馈的“末端 full path 没对接上 / 末端 XY 误差大”已经可以稳定复现为 **显著缩进**；目前未再观察到回到 `d≈12~15m` 的尾段坏例。

## 3) 下一步准备做什么
- 如果要继续把最终误差从 ~0.2m 拉到更小（比如 <=0.1m）：
  - 先把 `EXPERIMENT_DURATION_SEC` 再增加 5~10s 做确认（避免只是“收敛没跑完”）。
  - 再考虑把 `terminal_relative_position_` 的 completion 判据/hold 判据从“距离”改成同时包含 `|rel_x|/|rel_y|` 门限（避免 COMPELTED 过早）。

## 4) 最终要达成什么目标
- **M3 5/5 PASS**（指标同前）
- 同时保证末端 `trajectory_xy_full` 的两条轨迹能在尾段 **对接**（`rel_x/rel_y` 接近 0）。

## Evidence（最新验证）
- 单次 smoke：`results/20260407_211434_px4_sih`
  - `final_distance_m=0.214`
  - 末端 `rel_x≈-0.067m, rel_y≈-0.034m`（`docking_log.csv` 最后一行）
  - `trajectory_xy_full.png` 尾段已对接
- 5-run batch（均 `CARRIER_TRACKING_SPEED_LIMIT=13.0`）：
  - `20260407_211639_px4_sih`: `final_distance_m=0.212`, `hold_lat_0p2_zband_sec≈1.859`
  - `20260407_211832_px4_sih`: `final_distance_m=0.250`, `hold_lat_0p2_zband_sec≈0.600`
  - `20260407_212026_px4_sih`: `final_distance_m=0.214`, `hold_lat_0p2_zband_sec≈0.640`
  - `20260407_212216_px4_sih`: `final_distance_m=0.213`, `hold_lat_0p2_zband_sec≈0.440`
  - `20260407_212412_px4_sih`: `final_distance_m=0.200`, `hold_lat_0p2_zband_sec≈0.600`
