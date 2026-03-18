#ifndef EASYDOCKING_CONTROL__BACKSTEPPING_CONTROLLER_HPP_
#define EASYDOCKING_CONTROL__BACKSTEPPING_CONTROLLER_HPP_

#include <Eigen/Dense>

namespace easydocking
{

/**
 * @brief 反步控制算法
 * 
 * 实现自适应反步控制用于精确对接
 */
class BacksteppingController
{
public:
  BacksteppingController();
  ~BacksteppingController() = default;

  /**
   * @brief 设置控制参数
   */
  void setParameters(double k1, double k2, double adaptive_gain);

  /**
   * @brief 计算控制指令
   * @param position_error 位置误差
   * @param velocity_error 速度误差
   * @param desired_velocity 期望速度
   * @param dt 时间步长
   * @return 控制指令 (加速度)
   */
  Eigen::Vector3d computeControl(const Eigen::Vector3d& position_error,
                                const Eigen::Vector3d& velocity_error,
                                const Eigen::Vector3d& desired_velocity,
                                double dt);

  /**
   * @brief 获取干扰估计
   */
  Eigen::Vector3d getDisturbanceEstimate() const;

  /**
   * @brief 获取目标速度估计
   */
  Eigen::Vector3d getTargetVelocityEstimate() const;

  /**
   * @brief 重置自适应估计
   */
  void resetEstimates();

private:
  /**
   * @brief 更新自适应估计
   */
  void updateAdaptiveEstimates(const Eigen::Vector3d& position_error,
                              const Eigen::Vector3d& velocity_error,
                              double dt);

  /**
   * @brief 限制估计值范围
   */
  void clampEstimates();

  // 控制参数
  double k1_;  // 位置误差增益
  double k2_;  // 速度误差增益
  double adaptive_gain_;  // 自适应增益

  // 自适应估计
  Eigen::Vector3d disturbance_estimate_;
  Eigen::Vector3d target_velocity_estimate_;
};

}  // namespace easydocking

#endif  // EASYDOCKING_CONTROL__BACKSTEPPING_CONTROLLER_HPP_