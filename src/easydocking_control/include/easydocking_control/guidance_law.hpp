#ifndef EASYDOCKING_CONTROL__GUIDANCE_LAW_HPP_
#define EASYDOCKING_CONTROL__GUIDANCE_LAW_HPP_

#include <Eigen/Dense>

namespace easydocking
{

/**
 * @brief 制导律类
 * 
 * 实现各种制导算法用于无人机对接
 */
class GuidanceLaw
{
public:
  GuidanceLaw();
  ~GuidanceLaw() = default;

  /**
   * @brief 设置制导参数
   */
  void setParameters(double guidance_gain, double max_speed, double min_speed);

  /**
   * @brief 计算制导指令
   * @param relative_position 相对位置向量
   * @param relative_velocity 相对速度向量
   * @param relative_distance 相对距离
   * @return 期望速度指令
   */
  Eigen::Vector3d computeGuidanceCommand(const Eigen::Vector3d& relative_position,
                                        const Eigen::Vector3d& relative_velocity,
                                        double relative_distance);

  /**
   * @brief 计算比例导航制导
   * @param relative_position 相对位置向量
   * @param relative_velocity 相对速度向量
   * @param N 导航常数
   * @return 加速度指令
   */
  Eigen::Vector3d computePNGuidance(const Eigen::Vector3d& relative_position,
                                   const Eigen::Vector3d& relative_velocity,
                                   double N) const;

private:
  /**
   * @brief 计算期望接近速度
   */
  double computeDesiredApproachSpeed(double distance) const;

  /**
   * @brief 计算速度补偿
   */
  Eigen::Vector3d computeVelocityCompensation(const Eigen::Vector3d& relative_velocity,
                                             const Eigen::Vector3d& los_direction) const;

  // 制导参数
  double guidance_gain_;
  double max_approach_speed_;
  double min_approach_speed_;
};

}  // namespace easydocking

#endif  // EASYDOCKING_CONTROL__GUIDANCE_LAW_HPP_