#ifndef EASYDOCKING_CONTROL__DOCKING_CONTROLLER_HPP_
#define EASYDOCKING_CONTROL__DOCKING_CONTROLLER_HPP_

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <Eigen/Dense>
#include <array>
#include <string>
#include <memory>

#include "easydocking_control/relative_estimator.hpp"
#include "easydocking_control/guidance_law.hpp"
#include "easydocking_control/backstepping_controller.hpp"

namespace easydocking
{

/**
 * @brief 对接阶段枚举
 */
enum class DockingPhase {
  IDLE,           // 空闲
  TAKEOFF,        // 起飞至工作高度
  APPROACH,       // 接近阶段
  TRACKING,       // 追踪阶段
  DOCKING,        // 对接阶段
  COMPLETED,      // 完成
  FAILED          // 失败
};

/**
 * @brief 对接控制器类
 * 
 * 实现论文中的三阶段对接控制策略
 * 1. 接近阶段: 远距离接近
 * 2. 追踪阶段: 中距离跟踪
 * 3. 对接阶段: 近距离精确对接
 */
class DockingController
{
public:
  DockingController();
  ~DockingController() = default;

  // 参数设置
  void setParameters(double approach_dist, double tracking_dist, double docking_dist,
                    double k1, double k2, double adaptive_gain, double guidance_gain);
  void setIdleHoverAltitude(double altitude);
  void setPassiveTargetMode(bool enabled);
  void setDesiredRelativePosition(const Eigen::Vector3d & position);
  void setTerminalRelativePosition(const Eigen::Vector3d & position);
  void setCarrierLimits(double approach_speed, double tracking_speed, double docking_speed);
  void setCarrierMaxAccel(double max_accel);
  void setInterceptLookahead(double lookahead);
  void setDockingSpeedThreshold(double threshold);

  // 位姿更新
  void updatePoses(const geometry_msgs::msg::Pose& carrier_pose,
                  const geometry_msgs::msg::Twist& carrier_twist,
                  const geometry_msgs::msg::Pose& mini_pose,
                  const geometry_msgs::msg::Twist& mini_twist);

  // 控制命令
  void startDocking();
  void stopDocking();
  void reset();

  // 状态查询
  DockingPhase getCurrentPhase() const;
  bool isDockingActive() const;
  Eigen::Vector3d getRelativePosition() const;
  Eigen::Vector3d getRelativeVelocity() const;
  double getRelativeDistance() const;
  Eigen::Vector3d getCarrierVelocityCommand() const;
  Eigen::Vector3d getMiniVelocityCommand() const;
  Eigen::Vector3d getCarrierPositionSetpoint() const;
  Eigen::Vector3d getMiniPositionSetpoint() const;
  Eigen::Vector3d getDesiredRelativePosition() const;
  Eigen::Vector3d getTerminalRelativePosition() const;
  Eigen::Vector3d getDisturbanceEstimate() const;
  Eigen::Vector3d getEstimatedTargetVelocity() const;
  std::array<double, 20> getControllerDebug() const;
  std::string getCurrentPhaseName() const;

  // 计算控制指令
  void computeControlCommands(geometry_msgs::msg::Twist& carrier_cmd,
                             geometry_msgs::msg::Twist& mini_cmd);

private:
  // ========== 控制算法 ==========
  void takeoffPhaseControl(geometry_msgs::msg::Twist& carrier_cmd, geometry_msgs::msg::Twist& mini_cmd);
  void approachPhaseControl(geometry_msgs::msg::Twist& carrier_cmd, geometry_msgs::msg::Twist& mini_cmd);
  void trackingPhaseControl(geometry_msgs::msg::Twist& carrier_cmd, geometry_msgs::msg::Twist& mini_cmd);
  void dockingPhaseControl(geometry_msgs::msg::Twist& carrier_cmd, geometry_msgs::msg::Twist& mini_cmd);
  
  // 相对位姿估计
  void estimateRelativePose();

  // 阶段转换
  void transitionPhase();
  void applyCarrierAccelLimit(geometry_msgs::msg::Twist& carrier_cmd);

  // ========== 子模块 ==========
  std::unique_ptr<RelativeEstimator> relative_estimator_;
  std::unique_ptr<GuidanceLaw> guidance_law_;
  std::unique_ptr<BacksteppingController> backstepping_controller_;

  // ========== 成员变量 ==========
  // 当前位姿
  geometry_msgs::msg::Pose carrier_pose_;
  geometry_msgs::msg::Pose mini_pose_;
  geometry_msgs::msg::Twist carrier_twist_;
  geometry_msgs::msg::Twist mini_twist_;

  // 对接状态
  DockingPhase current_phase_;
  bool is_docking_active_;

  // 控制参数
  double approach_distance_;    // 接近阶段距离阈值 (m)
  double tracking_distance_;    // 追踪阶段距离阈值 (m)
  double docking_distance_;     // 对接阶段距离阈值 (m)
  double control_rate_;         // 控制频率 (Hz)

  // 控制增益
  double k1_;  // 位置误差增益
  double k2_;  // 速度误差增益
  double adaptive_gain_;  // 自适应增益
  double guidance_gain_;  // 制导增益

  // 指令缓存
  Eigen::Vector3d carrier_velocity_command_;
  Eigen::Vector3d mini_velocity_command_;
  Eigen::Vector3d carrier_position_setpoint_;
  Eigen::Vector3d mini_position_setpoint_;
  double idle_hover_altitude_;  // 空闲状态悬停高度
  Eigen::Vector3d desired_relative_position_;
  Eigen::Vector3d terminal_relative_position_;
  Eigen::Vector3d idle_carrier_hold_position_;
  Eigen::Vector3d idle_mini_hold_position_;
  bool idle_hold_initialized_;
  bool passive_target_mode_;
  bool rendezvous_corridor_initialized_;
  Eigen::Vector3d rendezvous_corridor_anchor_;
  Eigen::Vector3d rendezvous_corridor_axis_;
  double corridor_release_score_filtered_;
  bool corridor_release_armed_;
  int corridor_release_accept_counter_;
  double carrier_approach_speed_limit_;
  double carrier_tracking_speed_limit_;
  double carrier_docking_speed_limit_;
  double carrier_max_accel_;
  Eigen::Vector3d carrier_velocity_limited_;
  bool carrier_velocity_limited_initialized_;
  double intercept_lookahead_;
  double docking_speed_threshold_;
  int tracking_hold_counter_;
  int docking_hold_counter_;
  int completion_hold_counter_;
  int capture_hold_counter_;
  int passive_docking_entry_count_;
  bool passive_retry_used_;
  bool passive_retry_pending_second_entry_;
  double passive_retry_first_entry_lateral_abs_;
  double passive_retry_tracking_best_lateral_abs_;
  double min_docking_distance_;
  int terminal_z_hold_counter_;
  int terminal_z_out_counter_;
  int terminal_lat_hold_latch_counter_;
  int tracking_lat_hold_counter_;
  std::array<double, 20> controller_debug_;
};

}  // namespace easydocking

#endif  // EASYDOCKING_CONTROL__DOCKING_CONTROLLER_HPP_
