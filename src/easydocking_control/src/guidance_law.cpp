#include "easydocking_control/guidance_law.hpp"
#include <cmath>

namespace easydocking
{

GuidanceLaw::GuidanceLaw()
  : guidance_gain_(1.0),
    max_approach_speed_(2.0),
    min_approach_speed_(0.1)
{
}

void GuidanceLaw::setParameters(double guidance_gain, double max_speed, double min_speed)
{
  guidance_gain_ = guidance_gain;
  max_approach_speed_ = max_speed;
  min_approach_speed_ = min_speed;
}

Eigen::Vector3d GuidanceLaw::computeGuidanceCommand(const Eigen::Vector3d& relative_position,
                                                   const Eigen::Vector3d& relative_velocity,
                                                   double relative_distance)
{
  // 视线角制导 (Line-of-Sight Guidance)
  Eigen::Vector3d los_direction = -relative_position.normalized();

  // 计算期望接近速度 (基于距离的非线性函数)
  double desired_speed = computeDesiredApproachSpeed(relative_distance);

  // 基本视线角制导
  Eigen::Vector3d guidance_cmd = los_direction * desired_speed;

  // 添加相对速度补偿 (预测性制导)
  Eigen::Vector3d velocity_compensation = computeVelocityCompensation(relative_velocity, los_direction);

  guidance_cmd += guidance_gain_ * velocity_compensation;

  return guidance_cmd;
}

double GuidanceLaw::computeDesiredApproachSpeed(double distance) const
{
  // 距离越近，速度越慢 (非线性衰减)
  double speed = max_approach_speed_ * std::tanh(distance / 5.0);

  // 确保最小速度
  return std::max(speed, min_approach_speed_);
}

Eigen::Vector3d GuidanceLaw::computeVelocityCompensation(const Eigen::Vector3d& relative_velocity,
                                                        const Eigen::Vector3d& los_direction) const
{
  // 投影相对速度到视线方向
  double velocity_along_los = relative_velocity.dot(los_direction);

  // 如果目标在远离，增加接近速度
  // 如果目标在接近，减少接近速度
  Eigen::Vector3d compensation = -velocity_along_los * los_direction;

  return compensation;
}

Eigen::Vector3d GuidanceLaw::computePNGuidance(const Eigen::Vector3d& relative_position,
                                              const Eigen::Vector3d& relative_velocity,
                                              double N) const
{
  // 比例导航 (Proportional Navigation) 制导
  // a_cmd = N * V_rel * omega_los
  // 其中 omega_los = (V_rel × R) / |R|^2

  Eigen::Vector3d omega_los = relative_velocity.cross(relative_position) / (relative_position.squaredNorm());

  Eigen::Vector3d acceleration_cmd = N * relative_velocity.norm() * omega_los;

  return acceleration_cmd;
}

}  // namespace easydocking