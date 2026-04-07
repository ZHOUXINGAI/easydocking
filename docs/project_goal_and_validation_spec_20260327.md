# 空中异构对接项目目标与验证规范

## 1. 项目最终目的

### 1.1 任务形态
本项目的目标，不是简单复现一个“双多旋翼低速靠近”的演示，而是实现更接近“空中航母”概念的异构空中协同对接任务：

- `carrier`：大四轴/多旋翼，作为主动追接平台
- `mini`：固定翼，作为被接收子机
- 子机先完成起飞并进入稳定等待轨道
- 母机主动搜索、切入、追踪并建立末端对接窗口
- 末端阶段子机再有限度配合减速、直飞、对准与高度协调
- 一旦完成捕获，两机视为一个整体，后续只关注 `carrier` 的整体动力学

### 1.2 算法目标
核心目标不是某一个单独控制器最优，而是建立一套**分阶段协同框架**，统一解决以下问题：

1. `何时追`：选择可追窗口，而不是盲目提前释放
2. `如何切出等待轨道`：固定翼应沿等待轨道切线自然退出，而不是 inward spiral
3. `如何追近`：carrier 主动追接，fixed-wing 在非末段尽量保持被动
4. `何时 handoff`：何时从“被动切线直飞”切到“末段协同对接”
5. `如何末端同步`：在 docking 阶段同时压距离误差、相对速度、航向差和高度差
6. `何时判定成功`：不是擦肩而过，而是进入稳定捕获/刚性附着语义

### 1.3 当前工程边界
当前工程重点是从 mock/简化模型迁移到：

- `PX4` 固定翼 `SIH/SITL`
- `ROS 2` 通信与状态机
- `RViz` 可视化
- 自动出图、日志、批量验证

当前并不等价于“最终真实空中航母系统”。现阶段允许的工程边界是：

- 先把状态语义、轨迹语义、切换逻辑做对
- 再逐步压缩末端误差到最终目标
- 先保证行为正确，再追求论文式极限指标

---

## 2. 任务状态语义

### 2.1 系统级阶段语义
建议统一按下面语义理解系统：

- `WAIT_ORBIT`
  - `mini` 起飞后进入定高、定半径等待轨道
  - `carrier` 未进入有效追接窗口前，不应逼迫 `mini` 提前配合
- `RELEASE / TANGENT_EXIT`
  - `mini` 从等待轨道释放后，应沿当前轨道切线方向被动直飞
  - 这不是装饰动作，而是一个真实的中间阶段
  - 在这一阶段，`carrier` 是主动追接主体
- `HANDOFF`
  - 当几何关系真正转好，或到达明确 handoff 条件时，系统才允许从切线直飞段切到末段协同
- `TRACKING`
  - `carrier` 主要负责把横向、纵向和速度差压小
  - `mini` 仍不应做大幅机动抢着贴甲板
- `DOCKING`
  - `mini` 才开始有限配合：减速、直飞、对准、降高
  - 目标是把相对误差和相对速度一起压到可捕获窗口
- `COMPLETED`
  - 进入稳定附着/刚性连接语义
  - 此后两机视为一个整体

### 2.2 关键状态原则

1. `WAIT_ORBIT` 阶段，`mini` 的主要目标是稳定，不是参与追逐
2. `RELEASE / TANGENT_EXIT` 阶段，`mini` 的主要目标是沿切线被动直飞，不是 inward spiral
3. `TRACKING` 阶段，主导权在 `carrier`
4. `DOCKING` 阶段，才允许 `mini` 主动配合末端速度和姿态同步
5. 若没有进入明确 handoff 条件，不应提前结束切线直飞段

---

## 3. 关键行为约束

### 3.1 等待轨道约束
在 `WAIT_ORBIT` 阶段，`mini` 必须满足：

- 定高盘旋，不得持续掉高到接近地面
- 半径应稳定在目标半径附近，不得快速缩圈或放飞
- 在未 handoff 前，不得主动改成朝甲板内拉的轨迹

### 3.2 切线释放约束
一旦 `mini` 从等待轨道释放：

- 释放后的第一目标是**沿当前等待轨道切线自然退出**
- `mini` 不应在释放后立即朝轨道内侧收圈
- `mini` 不应在释放后立即将航向快速拧向 `carrier`
- `mini` 不应因为当前甲板瞬时几何不利而立刻取消切线段

### 3.3 主被动关系约束

- `carrier` 是主动追接方
- `mini` 在 `WAIT_ORBIT` 和 `RELEASE / TANGENT_EXIT` 阶段是被动方
- `mini` 只有在 `DOCKING` 阶段才进入有限主动配合

### 3.4 末段协同约束
在 `DOCKING` 阶段，系统才允许：

- `mini` 直飞保持
- `mini` 收油门减速
- `mini` 与 `carrier` 高度协调
- 末段相对速度同步

换言之：

- `TRACKING` 之前，不应提前出现明显“对接配合动作”
- `DOCKING` 之后，才允许末端协同收口

---

## 4. 必须记录的指标

后续所有 runner / logger / report 至少应保留以下量。

### 4.1 轨迹与状态

- `carrier_x, carrier_y, carrier_z`
- `mini_x, mini_y, mini_z`
- `carrier_yaw_deg, mini_yaw_deg`
- `phase`
- `relative_distance`
- `rel_x, rel_y, rel_z`
- `rel_vx, rel_vy, rel_vz`

### 4.2 fixed-wing 等待轨道指标

- `mini_wait_orbit_radius_target_m`
- `mini_wait_orbit_radius_mean_m`
- `mini_wait_orbit_radius_std_m`
- `mini_wait_orbit_radius_abs_error_max_m`
- `mini_wait_altitude_abs_error_max_m`

### 4.3 切线释放专项指标

必须新增或持续保留：

- `t_release`
- `t_tangent_exit_arm`
- `t_tangent_exit_clear`
- `tangent_exit_duration_sec`
- `tangent_exit_clear_reason`
- `release_phase`
- `release_heading_deg`
- `release_speed_mps`
- `release_radius_m`
- `post_release_radius_min_1s`
- `post_release_radius_min_2s`
- `post_release_heading_change_1s_deg`
- `post_release_forward_progress_to_deck`
- `post_release_lateral_miss_to_deck`

### 4.4 handoff / docking 指标

- `t_handoff`
- `handoff_phase`
- `handoff_distance_xy`
- `handoff_lateral_error`
- `handoff_forward_progress`
- `t_docking_enter`
- `t_completed`
- `min_distance_m`
- `best_window_rel_speed_mps`
- `best_window_rel_vx_mps`
- `best_window_rel_vy_mps`
- `best_window_rel_vz_mps`

### 4.5 PX4 / fixed-wing 诊断指标

- `mini_px4_true_airspeed_mps`
- `mini_tecs_true_airspeed_filtered_mps`
- `mini_tecs_underspeed_ratio`
- `mini_tecs_throttle_sp`
- `mini_tecs_pitch_sp_deg`
- 高度源与参考高度相关日志

---

## 5. 可执行、可量化的验收清单

## 5.1 修改评估流程
每次控制律或状态机改动，都按下面规则验证：

- 默认批量样本数：`10`
- 成功标准：`10 次里至少 9 次通过`
- 早停标准：前面累计 `2 次 fail` 立即停止该批次
- 若批次失败：
  - 删除该失败分支产生的成功样本目录
  - 保留失败样本目录
  - 分析失败原因后回退到改动前基线

这条规则优先级高于“单次看起来还行”。

## 5.2 当前阶段的中间验收标准
这是“语义正确性”标准，不是最终 0.2 m 成功标准。

### A. WAIT_ORBIT 合格
满足以下条件才算等待轨道合格：

- `mini` 有足够多等待轨道样本
- 等待轨道半径标准差处于可接受范围
- 高度误差不出现持续失控
- 不出现明显持续缩圈或掉高到近地面

### B. RELEASE / TANGENT_EXIT 合格
释放后必须满足：

1. 日志中必须出现
   - `glide score release accepted`
   - `tangent exit armed`

2. 释放后前 `1.0 s` 内，`mini` 不得表现为 inward spiral
   建议量化为至少满足其一：
   - 半径最小值相对释放半径的减小量 `<= 0.5 m`
   - 航向变化保持在小角度范围内，不能出现明显朝轨道内侧急转

3. 若 `tangent exit` 在极短时间内被系统自己清掉，应判为语义失败
   - 例如：`tangent_exit_duration_sec < 1.0 s`
   - 或者清除原因是“当前甲板前向投影为负”这一类瞬时几何判据

4. 释放后前 `1~2 s` 内，mini 的轨迹应近似直线外切，而不是明显朝圆心方向收口

### C. TRACKING 合格
满足以下条件才算 tracking 有效：

- `carrier` 能持续把距离压近，而不是只在远处绕飞
- 轨迹主导权应在 `carrier`，不是 `mini` 抢着大幅改轨迹
- 相对横向误差和相对速度开始下降

### D. DOCKING 合格
满足以下条件才算进入有效 docking：

- `phase` 真实进入 `DOCKING`
- `mini` 开始有限主动配合：减速、直飞、降高
- 相对距离继续下降，而不是高速擦肩而过

## 5.3 最终目标验收标准
这是最终目标，不要求当前版本立刻达成，但必须作为最终判据保留。

### 最终成功定义
只有同时满足下列条件，才算“对接成功”：

1. 相对位置误差收敛到 `0.2 m` 级别
2. 相对速度足够小，接近速度同步
3. 航向基本一致
4. 高度差在可附着窗口内
5. 该状态保持一段连续时间，而不是单帧擦过
6. 可进入 `COMPLETED` / 刚性附着语义

### 最终失败定义
以下任一情况都应判失败：

- `mini` 在 release 后 inward spiral
- `mini` 没有沿切线自然退出，而是马上朝甲板内拉
- `carrier` 没有成为主动追接主体
- 末段虽然最小距离很小，但相对速度仍很大，只是高速擦肩
- 进入 `DOCKING` 但没有形成稳定捕获窗口
- 没有达到 `COMPLETED`

---

## 6. 当前最重要的监督性要求

后续所有改动，不能只回答“最近点更小了没有”，还必须回答下面这些问题：

1. `mini` release 后是否真的沿切线被动直飞？
2. 切线段是否只是装饰动作，还是一个真实状态？
3. `carrier` 是否是主动追接主体？
4. handoff 是否有明确条件，而不是隐式乱切？
5. `DOCKING` 是否只在末段才允许 fixed-wing 主动配合？
6. 最近点变小，究竟是语义变对了，还是只是碰巧擦得更近？

如果这些问题答不清，即使 `min_distance_m` 好看，也不应判这版为成功。

---

## 7. 当前推荐的下一步验证重点

基于当前仓库状态，下一轮最优先验证的不是再盲调最小距离，而是：

1. `tangent_exit_duration_sec` 是否足够长，且不是立即被清掉
2. release 后前 `1~2 s` 的半径漂移和航向变化是否符合“沿切线直飞”语义
3. `carrier` 是否能在 mini 被动直飞阶段真正追近，而不是依赖 mini 提前内转配合
4. handoff 是否发生在明确的几何窗口内
5. 只有在这些语义都成立后，再继续压 `2 m -> 0.2 m`
