# PX4 SIH Repro Guide (Mini Wait Orbit)

## 目标
在新机器上从 `clone` 开始，复现 **mini 等待盘旋稳定**（先不关注 docking 通过率）。

## 1) 必备对齐文件
把下面文件放到同一目录（示例：`/home/hw/easydocking/docs/repo`）：

- `repro_bundle_20260408_011039_px4_sih_20260408_222400.tgz`
- `repro_bundle_20260408_011039_px4_sih_20260408_222400.tgz.sha256`
- `px4_align_20260408_225550.tgz`
- `px4_align_20260408_225550.tgz.sha256`
- `10040_sihsim_quadx`
- `10041_sihsim_airplane`

## 2) 仓库与依赖基线

```bash
cd /home/hw
git clone <your-easydocking-repo-url> easydocking
cd /home/hw/easydocking
git submodule update --init --recursive

# easydocking 目标 commit（复现基线）
git checkout bdfd02ea2af6b3da1382010c934012634df9bc86

# px4_msgs 子模块基线
git -C src/px4_msgs checkout 51e66788d6d538c7eabdb6a3becb6eee78167cb2
```

## 3) 校验对齐包

```bash
cd /home/hw/easydocking/docs/repo
sha256sum repro_bundle_20260408_011039_px4_sih_20260408_222400.tgz
sha256sum px4_align_20260408_225550.tgz
```

应与对应 `.sha256` 内的哈希一致。

## 4) 对齐 PX4 固件（关键）
> 复现成功依赖 PX4 特定基线：`95355590250768fd1a10ff84e25503c81a8536ed` + 本地 patch。

```bash
# 解包
mkdir -p /tmp/px4_align_20260408_225550
tar -xzf /home/hw/easydocking/docs/repo/px4_align_20260408_225550.tgz -C /tmp/px4_align_20260408_225550

cd /home/hw/PX4-Autopilot

# 切到基线 commit（detached HEAD）
git restore --worktree --staged .
git checkout --detach 95355590250768fd1a10ff84e25503c81a8536ed
git reset --hard

# 打上原机本地 patch
git apply --index --reject /tmp/px4_align_20260408_225550/px4_align_20260408_225550/px4_local_changes_953555902507.patch

# 对齐 submodules
git submodule update --init --recursive

# 覆盖 airframe（以导出文件为准）
cp -f /home/hw/easydocking/docs/repo/10040_sihsim_quadx \
  /home/hw/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/10040_sihsim_quadx
cp -f /home/hw/easydocking/docs/repo/10041_sihsim_airplane \
  /home/hw/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/10041_sihsim_airplane
```

## 5) 对齐 PX4 运行参数（parameters.bson）

```bash
# 从 repro bundle 解包（若尚未解）
mkdir -p /tmp/repro_bundle_20260408_011039
tar -xzf /home/hw/easydocking/docs/repo/repro_bundle_20260408_011039_px4_sih_20260408_222400.tgz -C /tmp/repro_bundle_20260408_011039

# 覆盖 rootfs 参数文件
cp -f /tmp/repro_bundle_20260408_011039/repro_bundle_20260408_011039_px4_sih_20260408_222400/px4_rootfs/1/parameters.bson \
  /home/hw/PX4-Autopilot/build/px4_sitl_default/rootfs/1/parameters.bson
cp -f /tmp/repro_bundle_20260408_011039/repro_bundle_20260408_011039_px4_sih_20260408_222400/px4_rootfs/2/parameters.bson \
  /home/hw/PX4-Autopilot/build/px4_sitl_default/rootfs/2/parameters.bson
```

## 6) 编译

```bash
# 编译 PX4
cd /home/hw/PX4-Autopilot
make px4_sitl_default -j8

# 编译 ROS 包
cd /home/hw/easydocking
source /opt/ros/humble/setup.bash
colcon build --packages-select easydocking_control
```

## 7) 运行复现实验（mini-only 验证）

```bash
cd /home/hw/easydocking
START_DELAY=never \
EXPERIMENT_DURATION_SEC=60 \
MINI_USE_OFFBOARD_ORBIT_HOLD=false \
MINI_ALLOW_ORBIT_RECENTERING=false \
MINI_LOITER_SPEED_COMMAND=10.5 \
MINI_ORBIT_SPEED=10.0 \
MINI_TRACKING_SPEED_COMMAND=9.2 \
MINI_DOCKING_SPEED_COMMAND=8.0 \
MINI_CAPTURE_SPEED_COMMAND=6.0 \
./scripts/run_px4_sih_docking_experiment.sh
```

## 8) 验证指标（summary.txt）
运行完成后检查结果目录 `results/<run_id>_px4_sih/summary.txt`，重点看：

- `mini_wait_orbit_radius_mean_m`（目标接近 80）
- `mini_wait_orbit_radius_abs_error_mean_m`（目标小，约 1~2）
- `mini_wait_altitude_error_mean_m`（目标接近导出基线，约 3~4）

参考基线（导出包）：
- `mini_wait_orbit_radius_mean_m=80.005`
- `mini_wait_orbit_radius_abs_error_mean_m=1.049`
- `mini_wait_altitude_error_mean_m=3.775`

## 9) 常见失败原因

- 仅对齐 easydocking 仓库，未对齐 PX4 commit + patch。
- `parameters.bson` 未覆盖，仍使用本机历史参数。
- 启动命令环境变量不一致（尤其 `MINI_*` / `START_DELAY`）。
- PX4 submodules 未同步到对应 commit。

## 10) 当前已验证复现结果
在本机按本流程执行，得到：
- `results/20260408_230150_px4_sih`
- `mini_wait_orbit_radius_mean_m=80.098`
- `mini_wait_orbit_radius_abs_error_mean_m=1.370`
- `mini_wait_altitude_error_mean_m=3.750`

说明 mini 等待盘旋已与参考包对齐复现。

## 11) 风扰动/阵风评估（Sim-to-Real 第一步）

单次风场注入示例（稳态风 + 随机阵风 + 突变风）：

```bash
cd /home/hw/easydocking
ENABLE_WIND_PROFILE=true \
WIND_BASE_N=2.0 \
WIND_BASE_E=-1.0 \
WIND_GUST_ENABLE=true \
WIND_GUST_AMP=1.5 \
WIND_GUST_CHANGE_SEC=1.5 \
WIND_STEP_EVENTS="35:2.0,-1.5;60:-2.5,2.0" \
./scripts/run_px4_sih_docking_experiment.sh
```

批量评估（分级风场）：

```bash
cd /home/hw/easydocking
RUNS_PER_LEVEL=2 ./scripts/run_px4_sih_wind_robustness_batch.sh
```

95/99 分位统计：

```bash
cd /home/hw/easydocking
python3 scripts/evaluate_wind_robustness.py --latest 6
```
