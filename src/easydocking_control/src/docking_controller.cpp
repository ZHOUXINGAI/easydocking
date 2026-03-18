#include "easydocking_control/docking_controller.hpp"
#include "easydocking_control/relative_estimator.hpp"
#include "easydocking_control/guidance_law.hpp"
#include "easydocking_control/backstepping_controller.hpp"
#include <cmath>
#include <memory>
#include <limits>

namespace easydocking
{

namespace
{

Eigen::Vector3d poseToEigen(const geometry_msgs::msg::Pose & pose)
{
  return Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
}

Eigen::Vector3d clampNorm(const Eigen::Vector3d & vector, double max_norm)
{
  const double norm = vector.norm();
  if (norm < 1e-6 || norm <= max_norm) {
    return vector;
  }
  return vector / norm * max_norm;
}

double clampValue(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(max_value, value));
}

Eigen::Vector3d carrierVelocity(const geometry_msgs::msg::Twist & twist)
{
  return Eigen::Vector3d(twist.linear.x, twist.linear.y, twist.linear.z);
}

Eigen::Vector3d computeLosDirection(const Eigen::Vector3d & vector, const Eigen::Vector3d & fallback)
{
  if (vector.norm() > 1e-6) {
    return vector.normalized();
  }
  if (fallback.norm() > 1e-6) {
    return fallback.normalized();
  }
  return Eigen::Vector3d::UnitX();
}

Eigen::Vector3d horizontalOnly(const Eigen::Vector3d & vector)
{
  return Eigen::Vector3d(vector.x(), vector.y(), 0.0);
}

}  // namespace

DockingController::DockingController()
  : relative_estimator_(std::make_unique<RelativeEstimator>()),
    guidance_law_(std::make_unique<GuidanceLaw>()),
    backstepping_controller_(std::make_unique<BacksteppingController>()),
    current_phase_(DockingPhase::IDLE),
    is_docking_active_(false),
    carrier_velocity_command_(Eigen::Vector3d::Zero()),
    mini_velocity_command_(Eigen::Vector3d::Zero()),
    carrier_position_setpoint_(Eigen::Vector3d::Zero()),
    mini_position_setpoint_(Eigen::Vector3d::Zero()),
    idle_hover_altitude_(1.5),
    desired_relative_position_(Eigen::Vector3d(0.0, 0.0, 0.6)),
    terminal_relative_position_(Eigen::Vector3d(0.0, 0.0, 0.2)),
    idle_carrier_hold_position_(Eigen::Vector3d::Zero()),
    idle_mini_hold_position_(Eigen::Vector3d::Zero()),
    idle_hold_initialized_(false),
    passive_target_mode_(false),
    carrier_approach_speed_limit_(3.0),
    carrier_tracking_speed_limit_(2.2),
    carrier_docking_speed_limit_(1.0),
    intercept_lookahead_(1.5),
    docking_speed_threshold_(0.8),
    docking_hold_counter_(0),
    completion_hold_counter_(0),
    capture_hold_counter_(0),
    min_docking_distance_(std::numeric_limits<double>::infinity())
{
  // 初始化参数
  approach_distance_ = 5.0;
  tracking_distance_ = 2.0;
  docking_distance_ = 0.5;
  control_rate_ = 50.0;
  k1_ = 1.0;
  k2_ = 2.0;
  adaptive_gain_ = 0.5;
  guidance_gain_ = 1.0;

  // 设置子模块参数
  backstepping_controller_->setParameters(k1_, k2_, adaptive_gain_);
  guidance_law_->setParameters(guidance_gain_, 2.0, 0.1);
}

void DockingController::setParameters(double approach_dist, double tracking_dist, double docking_dist,
                                    double k1, double k2, double adaptive_gain, double guidance_gain)
{
  approach_distance_ = approach_dist;
  tracking_distance_ = tracking_dist;
  docking_distance_ = docking_dist;
  k1_ = k1;
  k2_ = k2;
  adaptive_gain_ = adaptive_gain;
  guidance_gain_ = guidance_gain;

  // 更新子模块参数
  backstepping_controller_->setParameters(k1_, k2_, adaptive_gain_);
  guidance_law_->setParameters(guidance_gain_, 2.0, 0.1);
}

void DockingController::setIdleHoverAltitude(double altitude)
{
  idle_hover_altitude_ = altitude;
}

void DockingController::setPassiveTargetMode(bool enabled)
{
  passive_target_mode_ = enabled;
}

void DockingController::setDesiredRelativePosition(const Eigen::Vector3d & position)
{
  desired_relative_position_ = position;
}

void DockingController::setTerminalRelativePosition(const Eigen::Vector3d & position)
{
  terminal_relative_position_ = position;
}

void DockingController::setCarrierLimits(
  double approach_speed, double tracking_speed, double docking_speed)
{
  carrier_approach_speed_limit_ = approach_speed;
  carrier_tracking_speed_limit_ = tracking_speed;
  carrier_docking_speed_limit_ = docking_speed;
}

void DockingController::setInterceptLookahead(double lookahead)
{
  intercept_lookahead_ = lookahead;
}

void DockingController::setDockingSpeedThreshold(double threshold)
{
  docking_speed_threshold_ = threshold;
}

void DockingController::updatePoses(const geometry_msgs::msg::Pose& carrier_pose,
                                   const geometry_msgs::msg::Twist& carrier_twist,
                                   const geometry_msgs::msg::Pose& mini_pose,
                                   const geometry_msgs::msg::Twist& mini_twist)
{
  carrier_pose_ = carrier_pose;
  carrier_twist_ = carrier_twist;
  mini_pose_ = mini_pose;
  mini_twist_ = mini_twist;

  // 更新相对位姿估计器
  relative_estimator_->updateEstimates(carrier_pose, carrier_twist, mini_pose, mini_twist);
}

void DockingController::startDocking()
{
  if (is_docking_active_) {
    return;
  }
  is_docking_active_ = true;
  current_phase_ = passive_target_mode_ ? DockingPhase::APPROACH : DockingPhase::TAKEOFF;
  idle_hold_initialized_ = false;
  docking_hold_counter_ = 0;
  completion_hold_counter_ = 0;
  capture_hold_counter_ = 0;
  min_docking_distance_ = std::numeric_limits<double>::infinity();
  backstepping_controller_->resetEstimates();
}

void DockingController::stopDocking()
{
  is_docking_active_ = false;
  current_phase_ = DockingPhase::IDLE;
  idle_hold_initialized_ = false;
  docking_hold_counter_ = 0;
  completion_hold_counter_ = 0;
  capture_hold_counter_ = 0;
  min_docking_distance_ = std::numeric_limits<double>::infinity();
}

void DockingController::reset()
{
  current_phase_ = DockingPhase::IDLE;
  is_docking_active_ = false;
  idle_hold_initialized_ = false;
  docking_hold_counter_ = 0;
  completion_hold_counter_ = 0;
  capture_hold_counter_ = 0;
  min_docking_distance_ = std::numeric_limits<double>::infinity();
  backstepping_controller_->resetEstimates();
}

DockingPhase DockingController::getCurrentPhase() const
{
  return current_phase_;
}

bool DockingController::isDockingActive() const
{
  return is_docking_active_;
}

Eigen::Vector3d DockingController::getRelativePosition() const
{
  return relative_estimator_->getRelativePosition();
}

double DockingController::getRelativeDistance() const
{
  return relative_estimator_->getRelativeDistance();
}

void DockingController::computeControlCommands(geometry_msgs::msg::Twist& carrier_cmd,
                                              geometry_msgs::msg::Twist& mini_cmd)
{
  if (!is_docking_active_) {
    carrier_cmd = geometry_msgs::msg::Twist();
    mini_cmd = geometry_msgs::msg::Twist();
    carrier_velocity_command_.setZero();
    mini_velocity_command_.setZero();

    if (!idle_hold_initialized_) {
      idle_carrier_hold_position_ = poseToEigen(carrier_pose_);
      idle_mini_hold_position_ = poseToEigen(mini_pose_);
      idle_carrier_hold_position_.z() = idle_hover_altitude_;
      idle_mini_hold_position_.z() = idle_hover_altitude_;
      idle_hold_initialized_ = true;
    }

    carrier_position_setpoint_ = idle_carrier_hold_position_;
    mini_position_setpoint_ = passive_target_mode_ ? poseToEigen(mini_pose_) : idle_mini_hold_position_;
    return;
  }

  // 阶段转换判断
  transitionPhase();

  // 根据当前阶段执行控制
  switch (current_phase_) {
    case DockingPhase::TAKEOFF:
      takeoffPhaseControl(carrier_cmd, mini_cmd);
      break;
    case DockingPhase::APPROACH:
      approachPhaseControl(carrier_cmd, mini_cmd);
      break;
    case DockingPhase::TRACKING:
      trackingPhaseControl(carrier_cmd, mini_cmd);
      break;
    case DockingPhase::DOCKING:
      dockingPhaseControl(carrier_cmd, mini_cmd);
      break;
    case DockingPhase::COMPLETED:
      if (passive_target_mode_) {
        dockingPhaseControl(carrier_cmd, mini_cmd);
      } else {
        carrier_cmd = geometry_msgs::msg::Twist();
        mini_cmd = geometry_msgs::msg::Twist();
      }
      break;
    case DockingPhase::FAILED:
    default:
      carrier_cmd = geometry_msgs::msg::Twist();
      mini_cmd = geometry_msgs::msg::Twist();
      break;
  }
}

void DockingController::estimateRelativePose()
{
  // 相对位姿由relative_estimator_处理
}

void DockingController::transitionPhase()
{
  double distance = relative_estimator_->getRelativeDistance();
  const Eigen::Vector3d relative_position = relative_estimator_->getRelativePosition();
  const Eigen::Vector3d relative_velocity = relative_estimator_->getRelativeVelocity();
  const Eigen::Vector3d target_velocity(
    mini_twist_.linear.x, mini_twist_.linear.y, mini_twist_.linear.z);
  const Eigen::Vector3d terminal_error = relative_position - terminal_relative_position_;
  const Eigen::Vector3d track_direction =
    computeLosDirection(horizontalOnly(target_velocity), -horizontalOnly(terminal_error));
  const Eigen::Vector3d lateral_direction(-track_direction.y(), track_direction.x(), 0.0);
  const Eigen::Vector3d terminal_horizontal_error = horizontalOnly(terminal_error);
  const double terminal_along_error = terminal_horizontal_error.dot(track_direction);
  const double terminal_lateral_error = terminal_horizontal_error.dot(lateral_direction);
  const double along_relative_speed = horizontalOnly(relative_velocity).dot(track_direction);
  const double lateral_relative_speed = horizontalOnly(relative_velocity).dot(lateral_direction);
  const double docking_error =
    terminal_error.norm();
  const double relative_speed = relative_velocity.norm();
  const double carrier_altitude_error = std::abs(carrier_pose_.position.z - idle_hover_altitude_);
  const double mini_altitude_error = std::abs(mini_pose_.position.z - idle_hover_altitude_);
  const double docking_release_distance = std::max(tracking_distance_ * 2.4, 18.0);
  const double passive_docking_entry_distance =
    std::max(docking_distance_ * 1.6, 1.9);
  const double completion_distance_threshold = std::min(docking_distance_, 0.20);
  const bool passive_docking_window =
    docking_error < passive_docking_entry_distance &&
    std::abs(terminal_lateral_error) < 1.45 &&
    terminal_along_error > 0.20 &&
    terminal_along_error < 2.80 &&
    relative_position.z() > 0.60 &&
    relative_position.z() < 1.35 &&
    std::abs(lateral_relative_speed) < 2.4;
  const int completion_hold_steps = std::max(1, static_cast<int>(control_rate_ * 0.35));
  const int capture_hold_steps = std::max(2, static_cast<int>(control_rate_ * 0.06));
  const bool rigid_capture_envelope =
    passive_target_mode_ &&
    docking_error < 0.14 &&
    std::abs(terminal_along_error) < 0.10 &&
    std::abs(terminal_lateral_error) < 0.10 &&
    std::abs(relative_position.z() - terminal_relative_position_.z()) < 0.08 &&
    std::abs(along_relative_speed) < 0.45 &&
    std::abs(lateral_relative_speed) < 0.45 &&
    std::abs(relative_velocity.z()) < 0.4;
  const bool soft_capture_envelope =
    passive_target_mode_ &&
    docking_error < 0.18 &&
    terminal_along_error > -0.12 && terminal_along_error < 0.12 &&
    std::abs(terminal_lateral_error) < 0.14 &&
    std::abs(relative_position.z() - terminal_relative_position_.z()) < 0.08 &&
    std::abs(along_relative_speed) < 0.60 &&
    std::abs(lateral_relative_speed) < 0.60 &&
    std::abs(relative_velocity.z()) < 0.35;
  const bool capture_envelope =
    docking_error < 0.18 &&
    terminal_along_error > -0.12 && terminal_along_error < 0.12 &&
    std::abs(terminal_lateral_error) < 0.14 &&
    std::abs(relative_position.z() - terminal_relative_position_.z()) < 0.08 &&
    std::abs(along_relative_speed) < 0.65 &&
    std::abs(relative_velocity.z()) < 0.35 &&
    std::abs(lateral_relative_speed) < 0.65;

  switch (current_phase_) {
    case DockingPhase::TAKEOFF:
      docking_hold_counter_ = 0;
      completion_hold_counter_ = 0;
      capture_hold_counter_ = 0;
      if (carrier_altitude_error < 0.15 && (passive_target_mode_ || mini_altitude_error < 0.15)) {
        current_phase_ = DockingPhase::APPROACH;
      }
      break;

    case DockingPhase::APPROACH:
      docking_hold_counter_ = 0;
      completion_hold_counter_ = 0;
      capture_hold_counter_ = 0;
      if (distance < approach_distance_) {
        current_phase_ = DockingPhase::TRACKING;
      }
      break;

    case DockingPhase::TRACKING:
      docking_hold_counter_ = 0;
      completion_hold_counter_ = 0;
      capture_hold_counter_ = 0;
      if ((passive_target_mode_ && passive_docking_window) ||
        (!passive_target_mode_ && distance < tracking_distance_))
      {
        current_phase_ = DockingPhase::DOCKING;
        min_docking_distance_ = distance;
      } else if (distance > approach_distance_ * 1.2) {
        current_phase_ = DockingPhase::APPROACH;
      }
      break;

    case DockingPhase::DOCKING:
      docking_hold_counter_++;
      min_docking_distance_ = std::min(min_docking_distance_, distance);
      if (
        (docking_error < completion_distance_threshold && relative_speed < docking_speed_threshold_) ||
        capture_envelope)
      {
        completion_hold_counter_++;
      } else {
        completion_hold_counter_ = 0;
      }
      if (capture_envelope) {
        capture_hold_counter_++;
      } else {
        capture_hold_counter_ = 0;
      }

      if (
        completion_hold_counter_ >= completion_hold_steps ||
        capture_hold_counter_ >= capture_hold_steps ||
        rigid_capture_envelope ||
        soft_capture_envelope)
      {
        current_phase_ = DockingPhase::COMPLETED;
      } else if (
        docking_hold_counter_ > static_cast<int>(control_rate_ * 0.8) &&
        distance > docking_release_distance &&
        distance > min_docking_distance_ + 6.0)
      {
        current_phase_ = DockingPhase::TRACKING;
        docking_hold_counter_ = 0;
        completion_hold_counter_ = 0;
        capture_hold_counter_ = 0;
      }
      break;

    default:
      break;
  }
}

void DockingController::takeoffPhaseControl(
  geometry_msgs::msg::Twist& carrier_cmd,
  geometry_msgs::msg::Twist& mini_cmd)
{
  carrier_cmd = geometry_msgs::msg::Twist();
  mini_cmd = geometry_msgs::msg::Twist();
  carrier_velocity_command_.setZero();
  mini_velocity_command_.setZero();

  carrier_position_setpoint_ = poseToEigen(carrier_pose_);
  mini_position_setpoint_ = poseToEigen(mini_pose_);
  carrier_position_setpoint_.z() = idle_hover_altitude_;
  mini_position_setpoint_.z() = idle_hover_altitude_;
}

void DockingController::approachPhaseControl(geometry_msgs::msg::Twist& carrier_cmd,
                                            geometry_msgs::msg::Twist& mini_cmd)
{
  const Eigen::Vector3d relative_pos = relative_estimator_->getRelativePosition();
  const Eigen::Vector3d relative_error = relative_pos - desired_relative_position_;
  const Eigen::Vector3d target_velocity(
    mini_twist_.linear.x, mini_twist_.linear.y, mini_twist_.linear.z);
  const Eigen::Vector3d carrier_velocity = carrierVelocity(carrier_twist_);
  const Eigen::Vector3d los_direction = computeLosDirection(-horizontalOnly(relative_error), target_velocity);
  const double distance = relative_error.norm();
  const double lateral_error = std::abs(relative_error.y());
  const double along_track_error = relative_error.x();
  const double vertical_error = relative_error.z();

  if (passive_target_mode_) {
    // Approach: explicit LOS/intercept guidance to pull lateral geometry in before close tracking.
    const double pursuit_speed =
      clampValue(1.0 + 0.11 * distance + 0.05 * lateral_error, 1.4, carrier_approach_speed_limit_);
    const Eigen::Vector3d closing_velocity = los_direction * pursuit_speed;
    Eigen::Vector3d intercept_velocity = target_velocity + closing_velocity;
    intercept_velocity.z() = target_velocity.z() + clampValue(vertical_error * 0.7, -1.0, 1.0);

    const double preview_horizon =
      clampValue(intercept_lookahead_ * 0.7 + 0.025 * distance, 0.8, 1.8);
    const Eigen::Vector3d intercept_position =
      poseToEigen(mini_pose_) + target_velocity * preview_horizon - desired_relative_position_;

    Eigen::Vector3d velocity_correction(
      -0.06 * along_track_error,
      -0.26 * relative_error.y(),
      0.55 * vertical_error);
    const Eigen::Vector3d desired_velocity = intercept_velocity + velocity_correction;
    carrier_velocity_command_ =
      clampNorm(desired_velocity - 0.08 * (carrier_velocity - target_velocity),
      carrier_approach_speed_limit_);
    mini_velocity_command_.setZero();
    carrier_position_setpoint_ = intercept_position;
    mini_position_setpoint_ = poseToEigen(mini_pose_);
  } else {
    const Eigen::Vector3d desired_vel = clampNorm(-relative_error, 2.0);
    carrier_velocity_command_.setZero();
    mini_velocity_command_ = desired_vel;
    carrier_position_setpoint_ = poseToEigen(carrier_pose_);
    mini_position_setpoint_ = poseToEigen(mini_pose_) + desired_vel * 0.5;
  }

  carrier_cmd.linear.x = carrier_velocity_command_(0);
  carrier_cmd.linear.y = carrier_velocity_command_(1);
  carrier_cmd.linear.z = carrier_velocity_command_(2);
  carrier_cmd.angular.z = 0.0;

  mini_cmd.linear.x = mini_velocity_command_(0);
  mini_cmd.linear.y = mini_velocity_command_(1);
  mini_cmd.linear.z = mini_velocity_command_(2);
  mini_cmd.angular.z = 0.0;
}

void DockingController::trackingPhaseControl(geometry_msgs::msg::Twist& carrier_cmd,
                                            geometry_msgs::msg::Twist& mini_cmd)
{
  const Eigen::Vector3d relative_pos = relative_estimator_->getRelativePosition();
  const Eigen::Vector3d relative_vel = relative_estimator_->getRelativeVelocity();
  const Eigen::Vector3d relative_error = relative_pos - desired_relative_position_;
  const double distance = relative_error.norm();
  const Eigen::Vector3d target_velocity(
    mini_twist_.linear.x, mini_twist_.linear.y, mini_twist_.linear.z);
  const Eigen::Vector3d carrier_velocity = carrierVelocity(carrier_twist_);

  const Eigen::Vector3d guidance_cmd = guidance_law_->computeGuidanceCommand(
    relative_error, relative_vel, distance);

  if (passive_target_mode_) {
    // Tracking: carrier actively cuts in front of the fixed-wing path, while damping lateral miss distance.
    const Eigen::Vector3d track_direction =
      computeLosDirection(horizontalOnly(target_velocity), -horizontalOnly(relative_error));
    const Eigen::Vector3d lateral_direction(-track_direction.y(), track_direction.x(), 0.0);
    const double along_error = horizontalOnly(relative_error).dot(track_direction);
    const double lateral_error = horizontalOnly(relative_error).dot(lateral_direction);
    const double along_relative_speed = horizontalOnly(relative_vel).dot(track_direction);
    const double lateral_relative_speed = horizontalOnly(relative_vel).dot(lateral_direction);

    const double preview_horizon =
      clampValue(intercept_lookahead_ * 0.95 + 0.03 * distance + 0.08 * std::abs(lateral_error), 1.0, 2.6);
    const Eigen::Vector3d intercept_position =
      poseToEigen(mini_pose_) +
      target_velocity * preview_horizon +
      lateral_direction * clampValue(0.28 * lateral_error, -2.4, 2.4) -
      desired_relative_position_;

    const Eigen::Vector3d forward_intercept =
      track_direction * clampValue(
        0.22 * along_error + 0.52 * along_relative_speed + 0.10 * distance,
        -1.2,
        3.8);
    const Eigen::Vector3d lateral_intercept =
      lateral_direction * clampValue(
        0.46 * lateral_error + 0.88 * lateral_relative_speed,
        -4.2,
        4.2);
    const Eigen::Vector3d vertical_intercept(
      0.0,
      0.0,
      clampValue(0.58 * relative_error.z() + 0.34 * relative_vel.z(), -0.9, 0.9));
    const Eigen::Vector3d desired_velocity =
      target_velocity + forward_intercept + lateral_intercept + vertical_intercept - 0.35 * guidance_cmd;
    const Eigen::Vector3d smooth_velocity =
      desired_velocity - 0.10 * (carrier_velocity - target_velocity);
    carrier_velocity_command_ =
      clampNorm(smooth_velocity, carrier_tracking_speed_limit_);
    mini_velocity_command_.setZero();
    carrier_position_setpoint_ = intercept_position;
    mini_position_setpoint_ = poseToEigen(mini_pose_);
  } else {
    carrier_velocity_command_.setZero();
    mini_velocity_command_ = clampNorm(guidance_cmd, 1.2);
    carrier_position_setpoint_ = poseToEigen(carrier_pose_);
    mini_position_setpoint_ = poseToEigen(mini_pose_) + mini_velocity_command_ * 0.35;
  }

  carrier_cmd.linear.x = carrier_velocity_command_(0);
  carrier_cmd.linear.y = carrier_velocity_command_(1);
  carrier_cmd.linear.z = carrier_velocity_command_(2);
  carrier_cmd.angular.z = 0.0;

  mini_cmd.linear.x = mini_velocity_command_(0);
  mini_cmd.linear.y = mini_velocity_command_(1);
  mini_cmd.linear.z = mini_velocity_command_(2);
  mini_cmd.angular.z = 0.0;
}

void DockingController::dockingPhaseControl(geometry_msgs::msg::Twist& carrier_cmd,
                                           geometry_msgs::msg::Twist& mini_cmd)
{
  const Eigen::Vector3d relative_position = relative_estimator_->getRelativePosition();
  const Eigen::Vector3d terminal_error = relative_position - terminal_relative_position_;
  const Eigen::Vector3d vel_error = relative_estimator_->getRelativeVelocity();
  const Eigen::Vector3d target_velocity(
    mini_twist_.linear.x, mini_twist_.linear.y, mini_twist_.linear.z);
  const Eigen::Vector3d carrier_velocity = carrierVelocity(carrier_twist_);
  const Eigen::Vector3d track_direction =
    computeLosDirection(horizontalOnly(target_velocity), -horizontalOnly(terminal_error));
  const Eigen::Vector3d lateral_direction(-track_direction.y(), track_direction.x(), 0.0);
  const double terminal_distance = terminal_error.norm();
  const double terminal_lateral_error =
    horizontalOnly(terminal_error).dot(lateral_direction);
  const double far_blend = clampValue((terminal_distance - 0.35) / 2.0, 0.0, 1.0);
  const double lateral_hold_blend =
    clampValue((std::abs(terminal_lateral_error) - 0.18) / 0.55, 0.0, 1.0);
  const double staged_target_z =
    terminal_relative_position_.z() +
    (0.72 - terminal_relative_position_.z()) * std::max(far_blend, lateral_hold_blend);
  const double staged_along_target = 0.32 * far_blend;
  Eigen::Vector3d staged_target = terminal_relative_position_;
  staged_target.x() += track_direction.x() * staged_along_target;
  staged_target.y() += track_direction.y() * staged_along_target;
  staged_target.z() = staged_target_z;
  const Eigen::Vector3d pos_error = relative_position - staged_target;
  const double terminal_blend = clampValue((terminal_distance - 0.22) / 1.4, 0.0, 1.0);
  const Eigen::Vector3d desired_vel =
    Eigen::Vector3d(
      0.10 * target_velocity.x(),
      0.05 * target_velocity.y(),
      0.00 * target_velocity.z()) * terminal_blend;

  const Eigen::Vector3d control_input = backstepping_controller_->computeControl(
    pos_error, vel_error, desired_vel, 1.0 / control_rate_);

  if (passive_target_mode_) {
    // Docking: command carrier in the target track frame, with explicit relative-speed targets.
    const Eigen::Vector3d horizontal_error = horizontalOnly(pos_error);
    const Eigen::Vector3d horizontal_rel_velocity = horizontalOnly(vel_error);
    const double along_error = horizontal_error.dot(track_direction);
    const double lateral_error = horizontal_error.dot(lateral_direction);
    const double along_rel_speed = horizontal_rel_velocity.dot(track_direction);
    const double lateral_rel_speed = horizontal_rel_velocity.dot(lateral_direction);
    const double close_blend = clampValue((2.2 - terminal_distance) / 2.2, 0.0, 1.0);
    const double tight_blend = clampValue((0.9 - terminal_distance) / 0.9, 0.0, 1.0);

    const double desired_rel_along_limit =
      clampValue(1.35 - 0.78 * close_blend, 0.18, 1.35);
    const double desired_rel_lateral_limit =
      clampValue(1.00 - 0.64 * close_blend, 0.16, 1.00);
    const double desired_rel_vertical_limit =
      clampValue(0.42 - 0.22 * close_blend, 0.10, 0.42);

    const double desired_rel_along =
      clampValue(-0.92 * along_error, -desired_rel_along_limit, desired_rel_along_limit);
    const double desired_rel_lateral =
      clampValue(-1.22 * lateral_error, -desired_rel_lateral_limit, desired_rel_lateral_limit);
    const double desired_rel_vertical =
      clampValue(-0.98 * pos_error.z(), -desired_rel_vertical_limit, desired_rel_vertical_limit);

    const double along_speed_error = along_rel_speed - desired_rel_along;
    const double lateral_speed_error = lateral_rel_speed - desired_rel_lateral;
    const double vertical_speed_error = vel_error.z() - desired_rel_vertical;

    const double along_correction_limit =
      clampValue(2.8 - 1.4 * close_blend - 0.4 * tight_blend, 0.75, 2.8);
    const double lateral_correction_limit =
      clampValue(2.0 - 1.0 * close_blend - 0.3 * tight_blend, 0.45, 2.0);
    const double vertical_correction_limit =
      clampValue(0.72 - 0.34 * close_blend, 0.18, 0.72);

    const double along_correction =
      clampValue(
      1.04 * along_speed_error + 0.16 * along_error,
      -along_correction_limit,
      along_correction_limit);
    const double lateral_correction =
      clampValue(
      1.20 * lateral_speed_error + 0.26 * lateral_error,
      -lateral_correction_limit,
      lateral_correction_limit);
    const double vertical_correction =
      clampValue(
      0.96 * vertical_speed_error + 0.20 * pos_error.z(),
      -vertical_correction_limit,
      vertical_correction_limit);

    Eigen::Vector3d commanded_velocity =
      target_velocity +
      track_direction * along_correction +
      lateral_direction * lateral_correction +
      Eigen::Vector3d(0.0, 0.0, vertical_correction);
    commanded_velocity += control_input * (0.06 - 0.04 * close_blend);
    commanded_velocity -= 0.18 * (carrier_velocity - target_velocity);

    const double correction_speed_limit =
      clampValue(3.6 - 1.5 * close_blend - 0.5 * tight_blend, 0.9, 3.6);
    Eigen::Vector3d correction_velocity = commanded_velocity - target_velocity;
    correction_velocity = clampNorm(correction_velocity, correction_speed_limit);
    carrier_velocity_command_ =
      clampNorm(target_velocity + correction_velocity, carrier_docking_speed_limit_);
    mini_velocity_command_.setZero();
    const double preview_horizon = 0.24 + 0.16 * (1.0 - close_blend);
    const Eigen::Vector3d preview_offset =
      track_direction * desired_rel_along * preview_horizon +
      lateral_direction * desired_rel_lateral * preview_horizon +
      Eigen::Vector3d(0.0, 0.0, desired_rel_vertical * preview_horizon);
    carrier_position_setpoint_ =
      poseToEigen(mini_pose_) + target_velocity * preview_horizon * far_blend -
      staged_target - preview_offset;
    mini_position_setpoint_ = poseToEigen(mini_pose_);
  } else {
    carrier_velocity_command_.setZero();
    mini_velocity_command_ = clampNorm(control_input, 0.6);
    carrier_position_setpoint_ = poseToEigen(carrier_pose_);
    mini_position_setpoint_ = poseToEigen(carrier_pose_) + terminal_relative_position_;
  }

  carrier_cmd.linear.x = carrier_velocity_command_(0);
  carrier_cmd.linear.y = carrier_velocity_command_(1);
  carrier_cmd.linear.z = carrier_velocity_command_(2);
  carrier_cmd.angular.z = 0.0;

  mini_cmd.linear.x = mini_velocity_command_(0);
  mini_cmd.linear.y = mini_velocity_command_(1);
  mini_cmd.linear.z = mini_velocity_command_(2);
  mini_cmd.angular.z = 0.0;
}

Eigen::Vector3d DockingController::getRelativeVelocity() const
{
  return relative_estimator_->getRelativeVelocity();
}

Eigen::Vector3d DockingController::getCarrierVelocityCommand() const
{
  return carrier_velocity_command_;
}

Eigen::Vector3d DockingController::getMiniVelocityCommand() const
{
  return mini_velocity_command_;
}

Eigen::Vector3d DockingController::getCarrierPositionSetpoint() const
{
  return carrier_position_setpoint_;
}

Eigen::Vector3d DockingController::getMiniPositionSetpoint() const
{
  return mini_position_setpoint_;
}

Eigen::Vector3d DockingController::getDesiredRelativePosition() const
{
  return desired_relative_position_;
}

Eigen::Vector3d DockingController::getTerminalRelativePosition() const
{
  return terminal_relative_position_;
}

Eigen::Vector3d DockingController::getDisturbanceEstimate() const
{
  return backstepping_controller_->getDisturbanceEstimate();
}

Eigen::Vector3d DockingController::getEstimatedTargetVelocity() const
{
  return backstepping_controller_->getTargetVelocityEstimate();
}

std::string DockingController::getCurrentPhaseName() const
{
  switch (current_phase_) {
    case DockingPhase::IDLE:
      return "IDLE";
    case DockingPhase::TAKEOFF:
      return "TAKEOFF";
    case DockingPhase::APPROACH:
      return "APPROACH";
    case DockingPhase::TRACKING:
      return "TRACKING";
    case DockingPhase::DOCKING:
      return "DOCKING";
    case DockingPhase::COMPLETED:
      return "COMPLETED";
    case DockingPhase::FAILED:
      return "FAILED";
    default:
      return "UNKNOWN";
  }
}

}  // namespace easydocking
