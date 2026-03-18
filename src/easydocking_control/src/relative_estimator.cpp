#include "easydocking_control/relative_estimator.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

namespace easydocking
{

RelativeEstimator::RelativeEstimator()
  : relative_position_(Eigen::Vector3d::Zero()),
    relative_velocity_(Eigen::Vector3d::Zero()),
    relative_distance_(0.0)
{
}

void RelativeEstimator::updateEstimates(const geometry_msgs::msg::Pose& carrier_pose,
                                       const geometry_msgs::msg::Twist& carrier_twist,
                                       const geometry_msgs::msg::Pose& mini_pose,
                                       const geometry_msgs::msg::Twist& mini_twist)
{
  // 计算相对位置
  relative_position_(0) = mini_pose.position.x - carrier_pose.position.x;
  relative_position_(1) = mini_pose.position.y - carrier_pose.position.y;
  relative_position_(2) = mini_pose.position.z - carrier_pose.position.z;

  // 计算相对速度
  relative_velocity_(0) = mini_twist.linear.x - carrier_twist.linear.x;
  relative_velocity_(1) = mini_twist.linear.y - carrier_twist.linear.y;
  relative_velocity_(2) = mini_twist.linear.z - carrier_twist.linear.z;

  // 计算相对距离
  relative_distance_ = relative_position_.norm();

  // 计算相对姿态 (简化的)
  computeRelativeOrientation(carrier_pose, mini_pose);
}

void RelativeEstimator::computeRelativeOrientation(const geometry_msgs::msg::Pose& carrier_pose,
                                                  const geometry_msgs::msg::Pose& mini_pose)
{
  // 提取四元数
  tf2::Quaternion carrier_q(carrier_pose.orientation.x, carrier_pose.orientation.y,
                           carrier_pose.orientation.z, carrier_pose.orientation.w);
  tf2::Quaternion mini_q(mini_pose.orientation.x, mini_pose.orientation.y,
                        mini_pose.orientation.z, mini_pose.orientation.w);

  // 计算相对旋转
  tf2::Quaternion relative_q = carrier_q.inverse() * mini_q;

  // 转换为欧拉角 (ZYX)
  tf2::Matrix3x3 relative_matrix(relative_q);
  relative_matrix.getEulerYPR(relative_yaw_, relative_pitch_, relative_roll_);
}

Eigen::Vector3d RelativeEstimator::getRelativePosition() const
{
  return relative_position_;
}

Eigen::Vector3d RelativeEstimator::getRelativeVelocity() const
{
  return relative_velocity_;
}

double RelativeEstimator::getRelativeDistance() const
{
  return relative_distance_;
}

double RelativeEstimator::getRelativeYaw() const
{
  return relative_yaw_;
}

double RelativeEstimator::getRelativePitch() const
{
  return relative_pitch_;
}

double RelativeEstimator::getRelativeRoll() const
{
  return relative_roll_;
}

}  // namespace easydocking