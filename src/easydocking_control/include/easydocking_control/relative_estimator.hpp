#ifndef EASYDOCKING_CONTROL__RELATIVE_ESTIMATOR_HPP_
#define EASYDOCKING_CONTROL__RELATIVE_ESTIMATOR_HPP_

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <Eigen/Dense>

namespace easydocking
{

/**
 * @brief 相对位姿估计器
 * 
 * 计算两个无人机之间的相对位置、速度和姿态
 */
class RelativeEstimator
{
public:
  RelativeEstimator();
  ~RelativeEstimator() = default;

  /**
   * @brief 更新相对位姿估计
   */
  void updateEstimates(const geometry_msgs::msg::Pose& carrier_pose,
                      const geometry_msgs::msg::Twist& carrier_twist,
                      const geometry_msgs::msg::Pose& mini_pose,
                      const geometry_msgs::msg::Twist& mini_twist);

  /**
   * @brief 获取相对位置
   */
  Eigen::Vector3d getRelativePosition() const;

  /**
   * @brief 获取相对速度
   */
  Eigen::Vector3d getRelativeVelocity() const;

  /**
   * @brief 获取相对距离
   */
  double getRelativeDistance() const;

  /**
   * @brief 获取相对偏航角
   */
  double getRelativeYaw() const;

  /**
   * @brief 获取相对俯仰角
   */
  double getRelativePitch() const;

  /**
   * @brief 获取相对滚转角
   */
  double getRelativeRoll() const;

private:
  /**
   * @brief 计算相对姿态
   */
  void computeRelativeOrientation(const geometry_msgs::msg::Pose& carrier_pose,
                                 const geometry_msgs::msg::Pose& mini_pose);

  // 相对位姿
  Eigen::Vector3d relative_position_;
  Eigen::Vector3d relative_velocity_;
  double relative_distance_;

  // 相对姿态 (欧拉角)
  double relative_yaw_;
  double relative_pitch_;
  double relative_roll_;
};

}  // namespace easydocking

#endif  // EASYDOCKING_CONTROL__RELATIVE_ESTIMATOR_HPP_