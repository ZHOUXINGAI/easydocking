#include "easydocking_control/backstepping_controller.hpp"
#include <cmath>

namespace easydocking
{

BacksteppingController::BacksteppingController()
  : k1_(1.0),
    k2_(2.0),
    adaptive_gain_(0.5),
    disturbance_estimate_(Eigen::Vector3d::Zero()),
    target_velocity_estimate_(Eigen::Vector3d::Zero())
{
}

void BacksteppingController::setParameters(double k1, double k2, double adaptive_gain)
{
  k1_ = k1;
  k2_ = k2;
  adaptive_gain_ = adaptive_gain;
}

Eigen::Vector3d BacksteppingController::computeControl(const Eigen::Vector3d& position_error,
                                                     const Eigen::Vector3d& velocity_error,
                                                     const Eigen::Vector3d& desired_velocity,
                                                     double dt)
{
  // 虚拟控制量 (第一步)
  Eigen::Vector3d alpha = -k1_ * position_error + desired_velocity;

  // 计算虚拟控制误差
  Eigen::Vector3d z2 = velocity_error - alpha;

  // 实际控制律 (第二步)
  Eigen::Vector3d control_input = -k2_ * z2 - position_error - disturbance_estimate_;

  // 更新自适应估计
  updateAdaptiveEstimates(position_error, velocity_error, dt);

  return control_input;
}

void BacksteppingController::updateAdaptiveEstimates(const Eigen::Vector3d& position_error,
                                                   const Eigen::Vector3d& velocity_error,
                                                   double dt)
{
  // 自适应干扰估计 (基于位置误差)
  disturbance_estimate_ += adaptive_gain_ * position_error * dt;

  // 目标速度估计 (简化的)
  target_velocity_estimate_ += adaptive_gain_ * velocity_error * dt;

  // 限制估计值范围
  clampEstimates();
}

void BacksteppingController::clampEstimates()
{
  // 限制干扰估计的范围
  const double max_disturbance = 10.0;
  for (int i = 0; i < 3; ++i) {
    disturbance_estimate_(i) = std::max(-max_disturbance,
                                       std::min(max_disturbance, disturbance_estimate_(i)));
    target_velocity_estimate_(i) = std::max(-5.0,
                                           std::min(5.0, target_velocity_estimate_(i)));
  }
}

Eigen::Vector3d BacksteppingController::getDisturbanceEstimate() const
{
  return disturbance_estimate_;
}

Eigen::Vector3d BacksteppingController::getTargetVelocityEstimate() const
{
  return target_velocity_estimate_;
}

void BacksteppingController::resetEstimates()
{
  disturbance_estimate_.setZero();
  target_velocity_estimate_.setZero();
}

}  // namespace easydocking