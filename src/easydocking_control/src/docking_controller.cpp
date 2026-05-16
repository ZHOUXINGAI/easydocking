#include "easydocking_control/docking_controller.hpp"
#include "easydocking_control/relative_estimator.hpp"
#include "easydocking_control/guidance_law.hpp"
#include "easydocking_control/backstepping_controller.hpp"
#include <algorithm>
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

double phaseToCode(DockingPhase phase)
{
  switch (phase) {
    case DockingPhase::IDLE:
      return 0.0;
    case DockingPhase::TAKEOFF:
      return 1.0;
    case DockingPhase::APPROACH:
      return 2.0;
    case DockingPhase::TRACKING:
      return 3.0;
    case DockingPhase::DOCKING:
      return 4.0;
    case DockingPhase::COMPLETED:
      return 5.0;
    case DockingPhase::FAILED:
      return 6.0;
    default:
      return -1.0;
  }
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
    rendezvous_corridor_initialized_(false),
    rendezvous_corridor_anchor_(Eigen::Vector3d::Zero()),
    rendezvous_corridor_axis_(Eigen::Vector3d::UnitX()),
    corridor_release_score_filtered_(0.0),
    corridor_release_armed_(false),
    corridor_release_accept_counter_(0),
    corridor_plan_valid_(false),
    corridor_plan_published_(false),
    corridor_tangent_point_(Eigen::Vector3d::Zero()),
    corridor_carrier_target_(Eigen::Vector3d::Zero()),
    corridor_tangent_dir_(Eigen::Vector3d::Zero()),
    corridor_hold_duration_(0.0),
    corridor_planned_speed_(0.0),
    corridor_mini_arrival_delay_(0.0),
    carrier_approach_speed_limit_(3.0),
    carrier_tracking_speed_limit_(2.2),
    carrier_docking_speed_limit_(1.0),
    carrier_max_accel_(2.5),
    carrier_velocity_limited_(Eigen::Vector3d::Zero()),
    carrier_velocity_limited_initialized_(false),
    intercept_lookahead_(1.5),
    docking_speed_threshold_(0.8),
    tracking_hold_counter_(0),
    docking_hold_counter_(0),
    completion_hold_counter_(0),
    capture_hold_counter_(0),
    passive_docking_entry_count_(0),
    passive_retry_used_(false),
    passive_retry_pending_second_entry_(false),
    passive_retry_first_entry_lateral_abs_(std::numeric_limits<double>::infinity()),
    passive_retry_tracking_best_lateral_abs_(std::numeric_limits<double>::infinity()),
    min_docking_distance_(std::numeric_limits<double>::infinity()),
    terminal_z_hold_counter_(0),
    terminal_z_out_counter_(0),
    terminal_lat_hold_latch_counter_(0),
    tracking_lat_hold_counter_(0),
    departure_hold_active_(false),
    departure_hold_completed_(false),
    approach_hold_counter_(0),
    carrier_departure_min_orbit_fraction_(0.35),
    mini_orbit_radius_(0.0),
    mini_orbit_speed_(0.0),
    mini_orbit_center_(Eigen::Vector3d::Zero()),
    approach_direction_locked_(false),
    primary_approach_tangent_(Eigen::Vector3d::Zero())
{
  controller_debug_.fill(std::numeric_limits<double>::quiet_NaN());

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

void DockingController::setCarrierMaxAccel(double max_accel)
{
  carrier_max_accel_ = max_accel;
  carrier_velocity_limited_initialized_ = false;
  carrier_velocity_limited_.setZero();
}

void DockingController::setInterceptLookahead(double lookahead)
{
  intercept_lookahead_ = lookahead;
}

void DockingController::setDockingSpeedThreshold(double threshold)
{
  docking_speed_threshold_ = threshold;
}

void DockingController::setMiniOrbitModel(double radius, double speed, const Eigen::Vector3d & center)
{
  mini_orbit_radius_ = radius;
  mini_orbit_speed_ = speed;
  mini_orbit_center_ = center;
}

void DockingController::setMiniOrbitStartPhaseDeg(double phase_deg)
{
  mini_orbit_start_phase_deg_ = phase_deg;
}

void DockingController::setCarrierDepartureMinOrbitFraction(double fraction)
{
  carrier_departure_min_orbit_fraction_ = fraction;
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
  tracking_hold_counter_ = 0;
  completion_hold_counter_ = 0;
  capture_hold_counter_ = 0;
  passive_docking_entry_count_ = 0;
  passive_retry_used_ = false;
  passive_retry_pending_second_entry_ = false;
  passive_retry_first_entry_lateral_abs_ = std::numeric_limits<double>::infinity();
  passive_retry_tracking_best_lateral_abs_ = std::numeric_limits<double>::infinity();
  min_docking_distance_ = std::numeric_limits<double>::infinity();
  terminal_z_hold_counter_ = 0;
  terminal_z_out_counter_ = 0;
  terminal_lat_hold_latch_counter_ = 0;
  tracking_lat_hold_counter_ = 0;
  rendezvous_corridor_initialized_ = false;
  rendezvous_corridor_anchor_.setZero();
  rendezvous_corridor_axis_ = Eigen::Vector3d::UnitX();
  corridor_release_score_filtered_ = 0.0;
  corridor_release_armed_ = false;
  corridor_release_accept_counter_ = 0;
  corridor_plan_valid_ = false;
  corridor_plan_published_ = false;
  corridor_tangent_point_.setZero();
  corridor_carrier_target_.setZero();
  corridor_tangent_dir_.setZero();
  corridor_hold_duration_ = 0.0;
  corridor_planned_speed_ = 0.0;
  corridor_traj_start_.setZero();
  corridor_traj_end_.setZero();
  corridor_traj_duration_ = 0.0;
  corridor_traj_start_time_ = 0.0;
  corridor_traj_active_ = false;
  corridor_traj_counter_ = 0;
  corridor_arc_center_.setZero();
  corridor_arc_radius_ = 0.0;
  corridor_arc_phi_start_ = 0.0;
  corridor_arc_delta_phi_ = 0.0;
  departure_hold_active_ = false;
  departure_hold_completed_ = false;
  approach_hold_counter_ = 0;
  approach_direction_locked_ = false;
  primary_approach_tangent_.setZero();
  controller_debug_.fill(std::numeric_limits<double>::quiet_NaN());
  carrier_velocity_limited_initialized_ = false;
  carrier_velocity_limited_.setZero();
  backstepping_controller_->resetEstimates();
}

void DockingController::stopDocking()
{
  is_docking_active_ = false;
  current_phase_ = DockingPhase::IDLE;
  idle_hold_initialized_ = false;
  docking_hold_counter_ = 0;
  tracking_hold_counter_ = 0;
  completion_hold_counter_ = 0;
  capture_hold_counter_ = 0;
  passive_docking_entry_count_ = 0;
  passive_retry_used_ = false;
  passive_retry_pending_second_entry_ = false;
  passive_retry_first_entry_lateral_abs_ = std::numeric_limits<double>::infinity();
  passive_retry_tracking_best_lateral_abs_ = std::numeric_limits<double>::infinity();
  min_docking_distance_ = std::numeric_limits<double>::infinity();
  terminal_z_hold_counter_ = 0;
  terminal_z_out_counter_ = 0;
  terminal_lat_hold_latch_counter_ = 0;
  tracking_lat_hold_counter_ = 0;
  rendezvous_corridor_initialized_ = false;
  rendezvous_corridor_anchor_.setZero();
  rendezvous_corridor_axis_ = Eigen::Vector3d::UnitX();
  corridor_release_score_filtered_ = 0.0;
  corridor_release_armed_ = false;
  corridor_release_accept_counter_ = 0;
  corridor_plan_valid_ = false;
  corridor_plan_published_ = false;
  corridor_tangent_point_.setZero();
  corridor_carrier_target_.setZero();
  corridor_tangent_dir_.setZero();
  corridor_hold_duration_ = 0.0;
  corridor_planned_speed_ = 0.0;
  corridor_traj_start_.setZero();
  corridor_traj_end_.setZero();
  corridor_traj_duration_ = 0.0;
  corridor_traj_start_time_ = 0.0;
  corridor_traj_active_ = false;
  corridor_traj_counter_ = 0;
  corridor_arc_center_.setZero();
  corridor_arc_radius_ = 0.0;
  corridor_arc_phi_start_ = 0.0;
  corridor_arc_delta_phi_ = 0.0;
  departure_hold_active_ = false;
  departure_hold_completed_ = false;
  approach_hold_counter_ = 0;
  approach_direction_locked_ = false;
  primary_approach_tangent_.setZero();
  controller_debug_.fill(std::numeric_limits<double>::quiet_NaN());
  carrier_velocity_limited_initialized_ = false;
  carrier_velocity_limited_.setZero();
}

void DockingController::reset()
{
  current_phase_ = DockingPhase::IDLE;
  is_docking_active_ = false;
  idle_hold_initialized_ = false;
  docking_hold_counter_ = 0;
  tracking_hold_counter_ = 0;
  completion_hold_counter_ = 0;
  capture_hold_counter_ = 0;
  passive_docking_entry_count_ = 0;
  passive_retry_used_ = false;
  passive_retry_pending_second_entry_ = false;
  passive_retry_first_entry_lateral_abs_ = std::numeric_limits<double>::infinity();
  passive_retry_tracking_best_lateral_abs_ = std::numeric_limits<double>::infinity();
  min_docking_distance_ = std::numeric_limits<double>::infinity();
  terminal_z_hold_counter_ = 0;
  terminal_z_out_counter_ = 0;
  terminal_lat_hold_latch_counter_ = 0;
  tracking_lat_hold_counter_ = 0;
  rendezvous_corridor_initialized_ = false;
  rendezvous_corridor_anchor_.setZero();
  rendezvous_corridor_axis_ = Eigen::Vector3d::UnitX();
  corridor_release_score_filtered_ = 0.0;
  corridor_release_armed_ = false;
  corridor_release_accept_counter_ = 0;
  corridor_plan_valid_ = false;
  corridor_plan_published_ = false;
  corridor_tangent_point_.setZero();
  corridor_carrier_target_.setZero();
  corridor_tangent_dir_.setZero();
  corridor_hold_duration_ = 0.0;
  corridor_planned_speed_ = 0.0;
  corridor_traj_start_.setZero();
  corridor_traj_end_.setZero();
  corridor_traj_duration_ = 0.0;
  corridor_traj_start_time_ = 0.0;
  corridor_traj_active_ = false;
  corridor_traj_counter_ = 0;
  corridor_arc_center_.setZero();
  corridor_arc_radius_ = 0.0;
  corridor_arc_phi_start_ = 0.0;
  corridor_arc_delta_phi_ = 0.0;
  departure_hold_active_ = false;
  departure_hold_completed_ = false;
  approach_hold_counter_ = 0;
  approach_direction_locked_ = false;
  primary_approach_tangent_.setZero();
  controller_debug_.fill(std::numeric_limits<double>::quiet_NaN());
  carrier_velocity_limited_initialized_ = false;
  carrier_velocity_limited_.setZero();
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
      if (corridor_plan_valid_) {
        approachPhaseControl(carrier_cmd, mini_cmd);
      } else if (passive_target_mode_) {
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

  applyCarrierAccelLimit(carrier_cmd);
}

void DockingController::applyCarrierAccelLimit(geometry_msgs::msg::Twist& carrier_cmd)
{
  if (!std::isfinite(carrier_max_accel_) || carrier_max_accel_ <= 0.0) {
    carrier_velocity_command_ = Eigen::Vector3d(
      carrier_cmd.linear.x, carrier_cmd.linear.y, carrier_cmd.linear.z);
    carrier_velocity_limited_ = carrier_velocity_command_;
    carrier_velocity_limited_initialized_ = true;
    return;
  }

  const double dt = 1.0 / std::max(control_rate_, 1.0);
  const double max_delta = carrier_max_accel_ * dt;
  const Eigen::Vector3d desired(
    carrier_cmd.linear.x, carrier_cmd.linear.y, carrier_cmd.linear.z);

  if (!carrier_velocity_limited_initialized_) {
    carrier_velocity_limited_ = desired;
    carrier_velocity_limited_initialized_ = true;
  } else {
    Eigen::Vector3d delta = desired - carrier_velocity_limited_;
    // Limit acceleration per axis so a large lateral/vertical correction doesn't starve along-axis
    // tracking (which otherwise leads to the mini steadily pulling away and large terminal XY error).
    delta.x() = clampValue(delta.x(), -max_delta, max_delta);
    delta.y() = clampValue(delta.y(), -max_delta, max_delta);
    delta.z() = clampValue(delta.z(), -max_delta, max_delta);
    carrier_velocity_limited_ += delta;
  }

  carrier_cmd.linear.x = carrier_velocity_limited_.x();
  carrier_cmd.linear.y = carrier_velocity_limited_.y();
  carrier_cmd.linear.z = carrier_velocity_limited_.z();
  carrier_velocity_command_ = carrier_velocity_limited_;
}

void DockingController::estimateRelativePose()
{
  // 相对位姿由relative_estimator_处理
}

void DockingController::transitionPhase()
{
  // Corridor plan active — trajectory handles everything
  if (corridor_plan_valid_) return;

  double distance = relative_estimator_->getRelativeDistance();
  const Eigen::Vector3d relative_position = relative_estimator_->getRelativePosition();
  const Eigen::Vector3d relative_velocity = relative_estimator_->getRelativeVelocity();
  const Eigen::Vector3d target_velocity(
    mini_twist_.linear.x, mini_twist_.linear.y, mini_twist_.linear.z);
  const Eigen::Vector3d tracking_error = relative_position - desired_relative_position_;
  const Eigen::Vector3d terminal_error = relative_position - terminal_relative_position_;
  const Eigen::Vector3d track_direction =
    computeLosDirection(horizontalOnly(target_velocity), -horizontalOnly(terminal_error));
  const Eigen::Vector3d lateral_direction(-track_direction.y(), track_direction.x(), 0.0);
  const Eigen::Vector3d tracking_horizontal_error = horizontalOnly(tracking_error);
  const Eigen::Vector3d terminal_horizontal_error = horizontalOnly(terminal_error);
  const double tracking_horizontal_distance = tracking_horizontal_error.norm();
  const double tracking_along_error = tracking_horizontal_error.dot(track_direction);
  const double tracking_lateral_error = tracking_horizontal_error.dot(lateral_direction);
  const double terminal_along_error = terminal_horizontal_error.dot(track_direction);
  const double terminal_lateral_error = terminal_horizontal_error.dot(lateral_direction);
  const double along_relative_speed = horizontalOnly(relative_velocity).dot(track_direction);
  const double lateral_relative_speed = horizontalOnly(relative_velocity).dot(lateral_direction);
  const double docking_error =
    terminal_error.norm();
  const double relative_speed = relative_velocity.norm();
  const double carrier_altitude_error = std::abs(carrier_pose_.position.z - idle_hover_altitude_);
  const double mini_altitude_error = std::abs(mini_pose_.position.z - idle_hover_altitude_);
  const double docking_release_distance = passive_target_mode_
    ? std::max(tracking_distance_ * 3.6, 42.0)
    : std::max(tracking_distance_ * 2.4, 18.0);
  const double passive_docking_entry_distance =
    std::max(docking_distance_ * 1.6, 1.9);
  const double passive_terminal_entry_distance =
    std::max(docking_distance_ * 4.8, 6.2);
  const double terminal_z_band_min = 0.25;
  const double terminal_z_band_max = 0.95;
  const double completion_distance_threshold = std::min(docking_distance_, 0.20);
  const bool terminal_z_in_band =
    relative_position.z() > terminal_z_band_min &&
    relative_position.z() < terminal_z_band_max;
  if (terminal_z_in_band) {
    terminal_z_hold_counter_++;
    terminal_z_out_counter_ = 0;
  } else {
    terminal_z_out_counter_++;
    terminal_z_hold_counter_ = 0;
  }
  const int terminal_z_hold_steps = std::max(1, static_cast<int>(control_rate_ * 0.20));
  const bool terminal_z_hold_ready = terminal_z_hold_counter_ >= terminal_z_hold_steps;
  const int terminal_lat_hold_latch_steps =
    std::max(1, static_cast<int>(control_rate_ * 1.00));
  const bool terminal_lat_hold_condition =
    passive_target_mode_ &&
    terminal_z_in_band &&
    distance < 10.0 &&
    std::abs(terminal_along_error) < 3.2 &&
    std::abs(terminal_lateral_error) < 0.20;
  if (terminal_lat_hold_condition) {
    terminal_lat_hold_latch_counter_ = terminal_lat_hold_latch_steps;
  } else if (terminal_lat_hold_latch_counter_ > 0) {
    terminal_lat_hold_latch_counter_--;
  }
  const int tracking_lat_hold_steps =
    std::max(1, static_cast<int>(control_rate_ * 1.20));
  const bool tracking_lat_hold_condition =
    passive_target_mode_ &&
    terminal_z_in_band &&
    distance < 10.0 &&
    std::abs(terminal_along_error) < 3.2 &&
    std::abs(terminal_lateral_error) < 0.20;
  if (tracking_lat_hold_condition) {
    tracking_lat_hold_counter_ = tracking_lat_hold_steps;
  } else if (tracking_lat_hold_counter_ > 0) {
    tracking_lat_hold_counter_--;
  }
  const bool passive_tracking_window =
    passive_target_mode_ &&
    tracking_horizontal_distance < std::max(approach_distance_ * 1.75, 60.0) &&
    std::abs(tracking_along_error) < 30.0 &&
    std::abs(tracking_lateral_error) < 24.0 &&
    std::abs(tracking_error.z()) < 1.0 &&
    std::abs(along_relative_speed) < 8.0 &&
    std::abs(lateral_relative_speed) < 6.0 &&
    std::abs(relative_velocity.z()) < 1.8;
  const bool passive_tracking_hold_window =
    passive_target_mode_ &&
    tracking_horizontal_distance < std::max(approach_distance_ * 2.05, 78.0) &&
    std::abs(tracking_along_error) < 36.0 &&
    std::abs(tracking_lateral_error) < 28.0 &&
    std::abs(tracking_error.z()) < 1.4 &&
    std::abs(along_relative_speed) < 10.0 &&
    std::abs(lateral_relative_speed) < 8.0 &&
    std::abs(relative_velocity.z()) < 2.2;
  const bool passive_terminal_align_window =
    passive_target_mode_ &&
    tracking_horizontal_distance < std::max(approach_distance_ * 1.75, 60.0) &&
    std::abs(tracking_along_error) < 30.0 &&
    std::abs(tracking_lateral_error) < 26.0 &&
    std::abs(tracking_error.z()) < 0.9 &&
    relative_position.z() > terminal_z_band_min &&
    relative_position.z() < terminal_z_band_max &&
    std::abs(along_relative_speed) < 8.0 &&
    std::abs(lateral_relative_speed) < 6.2 &&
    std::abs(relative_velocity.z()) < 2.2;
  const bool passive_docking_window =
    docking_error < passive_docking_entry_distance &&
    std::abs(terminal_lateral_error) < 1.45 &&
    terminal_along_error > 0.20 &&
    terminal_along_error < 2.80 &&
    relative_position.z() > terminal_z_band_min &&
    relative_position.z() < terminal_z_band_max &&
    std::abs(lateral_relative_speed) < 2.4;
  const bool passive_terminal_window =
    passive_target_mode_ &&
    docking_error < passive_terminal_entry_distance &&
    std::abs(terminal_lateral_error) < 4.4 &&
    terminal_along_error > 0.45 &&
    terminal_along_error < 5.40 &&
    relative_position.z() > terminal_z_band_min &&
    relative_position.z() < terminal_z_band_max &&
    std::abs(along_relative_speed) < 6.0 &&
    std::abs(lateral_relative_speed) < 5.2 &&
    std::abs(relative_velocity.z()) < 2.4;
  const bool passive_terminal_commit_window =
    passive_target_mode_ &&
    docking_error < 3.2 &&
    std::abs(terminal_lateral_error) < 2.0 &&
    terminal_along_error > 0.35 &&
    terminal_along_error < 2.8 &&
    relative_position.z() > terminal_z_band_min &&
    relative_position.z() < terminal_z_band_max &&
    std::abs(along_relative_speed) < 2.0 &&
    std::abs(lateral_relative_speed) < 1.2 &&
    std::abs(relative_velocity.z()) < 3.2;
  const bool passive_terminal_prepare_window =
    passive_target_mode_ &&
    docking_error < passive_terminal_entry_distance + 0.8 &&
    std::abs(terminal_lateral_error) < 4.8 &&
    terminal_along_error > -0.40 &&
    terminal_along_error < 5.60 &&
    relative_position.z() > terminal_z_band_min &&
    relative_position.z() < terminal_z_band_max &&
    std::abs(along_relative_speed) < 6.5 &&
    std::abs(lateral_relative_speed) < 5.4 &&
    std::abs(relative_velocity.z()) < 2.5;
  const bool passive_terminal_progress_ok =
    passive_target_mode_ &&
    std::abs(terminal_along_error) < 3.8;
  // NOTE: We do not gate passive DOCKING entry purely on terminal lateral error.
  // Historical samples show the non-corridor docking controller can collapse large
  // lateral error after entering DOCKING; instead we selectively disable corridor-mode
  // docking when lateral error is still large (see dockingPhaseControl()).
  const int completion_hold_steps = std::max(1, static_cast<int>(control_rate_ * 0.35));
  const int capture_hold_steps = std::max(2, static_cast<int>(control_rate_ * 0.06));
  const int passive_docking_release_hold_steps = std::max(
    1, static_cast<int>(control_rate_ * 2.0));
  const int passive_docking_retry_hold_steps = std::max(
    1, static_cast<int>(control_rate_ * 0.45));
  const int passive_docking_retry_window_steps = std::max(
    1, static_cast<int>(control_rate_ * 2.40));
  // Keep a short settle for first entry so we do not miss the narrow docking window
  // at late intercept, and use a slightly longer settle for retry->second entry.
  const int passive_tracking_settle_steps_first = std::max(
    1, static_cast<int>(control_rate_ * 0.90));
  const int passive_tracking_settle_steps_retry = std::max(
    1, static_cast<int>(control_rate_ * 0.45));
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
  const bool passive_soft_attach_envelope =
    passive_target_mode_ &&
    docking_error < 1.25 &&
    std::abs(relative_position.z() - terminal_relative_position_.z()) < 0.35 &&
    relative_speed < 0.35 &&
    std::abs(lateral_relative_speed) < 0.30 &&
    std::abs(relative_velocity.z()) < 0.25;
  const bool passive_corridor_capture_envelope =
    passive_target_mode_ &&
    rendezvous_corridor_initialized_ &&
    docking_error < 2.0 &&
    terminal_along_error > 1.2 && terminal_along_error < 2.2 &&
    std::abs(terminal_lateral_error) < 0.15 &&
    std::abs(relative_position.z() - terminal_relative_position_.z()) < 0.12 &&
    std::abs(along_relative_speed) < 0.55 &&
    std::abs(lateral_relative_speed) < 0.25 &&
    std::abs(relative_velocity.z()) < 0.20;
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
      tracking_hold_counter_ = 0;
      docking_hold_counter_ = 0;
      completion_hold_counter_ = 0;
      capture_hold_counter_ = 0;
      if (carrier_altitude_error < 0.15 && (passive_target_mode_ || mini_altitude_error < 0.15)) {
        current_phase_ = DockingPhase::APPROACH;
      }
      break;

    case DockingPhase::APPROACH:
      tracking_hold_counter_ = 0;
      docking_hold_counter_ = 0;
      completion_hold_counter_ = 0;
      capture_hold_counter_ = 0;
      if (passive_target_mode_) {
        const bool passive_ready_for_terminal =
          terminal_z_hold_ready &&
          (passive_docking_window || passive_terminal_window ||
          passive_terminal_commit_window || passive_terminal_prepare_window);
        if (passive_ready_for_terminal) {
          // When cross-track error is still large, force a TRACKING corridor cycle first so
          // we can leverage the retry mechanism (DOCKING->TRACKING->DOCKING) to improve
          // intercept geometry before committing to terminal control.
          if (std::abs(tracking_lateral_error) > 1.2) {
            rendezvous_corridor_initialized_ = false;
            corridor_release_score_filtered_ = 0.0;
            corridor_release_armed_ = false;
            corridor_release_accept_counter_ = 0;
            tracking_hold_counter_ = 0;
            current_phase_ = DockingPhase::TRACKING;
            break;
          }
          rendezvous_corridor_initialized_ = false;
          corridor_release_score_filtered_ = 0.0;
          corridor_release_armed_ = false;
          corridor_release_accept_counter_ = 0;
          tracking_hold_counter_ = 0;
          current_phase_ = DockingPhase::DOCKING;
          passive_docking_entry_count_++;
          if (passive_target_mode_) {
            if (passive_docking_entry_count_ == 1) {
              passive_retry_first_entry_lateral_abs_ = std::abs(terminal_lateral_error);
            }
            passive_retry_pending_second_entry_ = false;
          }
          min_docking_distance_ = distance;
        } else if (passive_tracking_window) {
          // Use TRACKING to prepare a better tangent intercept when we are not yet
          // ready to enter passive docking.
          rendezvous_corridor_initialized_ = false;
          corridor_release_score_filtered_ = 0.0;
          corridor_release_armed_ = false;
          corridor_release_accept_counter_ = 0;
          tracking_hold_counter_ = 0;
          current_phase_ = DockingPhase::TRACKING;
        } else {
          const bool passive_force_first_entry_from_approach =
            passive_target_mode_ &&
            passive_docking_entry_count_ == 0 &&
            distance < std::max(passive_terminal_entry_distance + 2.4, 6.2) &&
            std::abs(terminal_lateral_error) < 3.6 &&
            terminal_along_error > -1.8 &&
            terminal_along_error < 6.5 &&
            relative_position.z() > terminal_z_band_min &&
            relative_position.z() < terminal_z_band_max &&
            std::abs(lateral_relative_speed) < 4.5 &&
            std::abs(relative_velocity.z()) < 3.5;
          if (passive_force_first_entry_from_approach) {
            rendezvous_corridor_initialized_ = false;
            corridor_release_score_filtered_ = 0.0;
            corridor_release_armed_ = false;
            corridor_release_accept_counter_ = 0;
            tracking_hold_counter_ = 0;
            // Avoid forcing DOCKING entry with large cross-track. It can take multiple seconds
            // to collapse >~2m lateral error and we often won't achieve a >=0.3s in-band hold
            // within the <=10m evaluation window. Prefer a TRACKING corridor cycle first.
            if (std::abs(tracking_lateral_error) > 1.2) {
              current_phase_ = DockingPhase::TRACKING;
            } else {
              current_phase_ = DockingPhase::DOCKING;
              passive_docking_entry_count_++;
              passive_retry_first_entry_lateral_abs_ = std::abs(terminal_lateral_error);
              passive_retry_pending_second_entry_ = false;
              min_docking_distance_ = distance;
            }
          }
        }
      } else if (distance < approach_distance_) {
        rendezvous_corridor_initialized_ = false;
        corridor_release_score_filtered_ = 0.0;
        corridor_release_armed_ = false;
        corridor_release_accept_counter_ = 0;
        current_phase_ = DockingPhase::TRACKING;
      }
      break;

    case DockingPhase::TRACKING: {
      tracking_hold_counter_++;
      docking_hold_counter_ = 0;
      completion_hold_counter_ = 0;
      capture_hold_counter_ = 0;
      if (passive_target_mode_ && passive_retry_pending_second_entry_) {
        passive_retry_tracking_best_lateral_abs_ = std::min(
          passive_retry_tracking_best_lateral_abs_, std::abs(terminal_lateral_error));
      }
      const bool passive_second_entry_lateral_ready =
        !passive_retry_pending_second_entry_ ||
        (std::isfinite(passive_retry_first_entry_lateral_abs_) &&
        std::isfinite(passive_retry_tracking_best_lateral_abs_) &&
        (
        passive_retry_tracking_best_lateral_abs_ <=
        passive_retry_first_entry_lateral_abs_ - 0.20 ||
        passive_retry_tracking_best_lateral_abs_ <= 1.80));
      const bool passive_ready_to_enter_docking =
        passive_target_mode_ &&
        passive_terminal_progress_ok &&
        terminal_z_hold_ready &&
        (tracking_horizontal_distance > 10.0 || std::abs(terminal_lateral_error) < 1.6) &&
        passive_second_entry_lateral_ready &&
        rendezvous_corridor_initialized_ && (
        passive_docking_window || passive_terminal_window ||
        passive_terminal_commit_window ||
        passive_terminal_prepare_window || passive_terminal_align_window);
      const bool passive_retry_second_entry_window =
        passive_target_mode_ &&
        passive_retry_pending_second_entry_ &&
        rendezvous_corridor_initialized_ &&
        terminal_z_hold_ready &&
        (tracking_horizontal_distance > 10.0 || std::abs(terminal_lateral_error) < 1.6) &&
        docking_error < passive_terminal_entry_distance + 0.8 &&
        std::abs(terminal_lateral_error) < 2.40 &&
        std::abs(terminal_lateral_error) <
        std::max(1.60, passive_retry_first_entry_lateral_abs_ - 0.15) &&
        terminal_along_error > -1.2 &&
        terminal_along_error < 6.4 &&
        relative_position.z() > terminal_z_band_min &&
        relative_position.z() < terminal_z_band_max &&
        std::abs(lateral_relative_speed) < 2.8 &&
        std::abs(relative_velocity.z()) < 2.4;
      const bool passive_retry_force_second_entry =
        passive_target_mode_ &&
        passive_retry_pending_second_entry_ &&
        rendezvous_corridor_initialized_ &&
        std::isfinite(passive_retry_first_entry_lateral_abs_) &&
        tracking_hold_counter_ > passive_tracking_settle_steps_retry &&
        terminal_z_hold_ready &&
        (tracking_horizontal_distance > 10.0 || std::abs(terminal_lateral_error) < 1.6) &&
        docking_error < passive_terminal_entry_distance + 2.0 &&
        std::abs(terminal_lateral_error) <
        std::max(1.60, passive_retry_first_entry_lateral_abs_ - 0.05) &&
        relative_position.z() > terminal_z_band_min &&
        relative_position.z() < terminal_z_band_max &&
        std::abs(lateral_relative_speed) < 3.5 &&
        std::abs(relative_velocity.z()) < 3.0;
      const bool passive_precision_reentry_window =
        passive_target_mode_ &&
        passive_docking_entry_count_ >= 2 &&
        !passive_retry_pending_second_entry_ &&
        terminal_z_hold_ready &&
        (tracking_horizontal_distance > 10.0 || std::abs(terminal_lateral_error) < 1.6) &&
        docking_error < passive_terminal_entry_distance + 2.6 &&
        std::abs(terminal_lateral_error) < 0.90 &&
        terminal_along_error > 2.0 &&
        terminal_along_error < 10.5 &&
        relative_position.z() > 0.25 &&
        relative_position.z() < 0.95 &&
        std::abs(lateral_relative_speed) < 1.4 &&
        std::abs(relative_velocity.z()) < 1.2;
      const bool passive_force_first_entry_window =
        passive_target_mode_ &&
        passive_docking_entry_count_ == 0 &&
        !passive_retry_pending_second_entry_ &&
        tracking_hold_counter_ > passive_tracking_settle_steps_first &&
        terminal_z_hold_ready &&
        (tracking_horizontal_distance > 10.0 || std::abs(terminal_lateral_error) < 1.6) &&
        docking_error < passive_terminal_entry_distance + 2.2 &&
        std::abs(terminal_lateral_error) < 3.6 &&
        relative_position.z() > terminal_z_band_min &&
        relative_position.z() < terminal_z_band_max &&
        std::abs(lateral_relative_speed) < 4.5 &&
        std::abs(relative_velocity.z()) < 3.5;
      const int passive_tracking_settle_steps =
        passive_retry_pending_second_entry_
        ? passive_tracking_settle_steps_retry
        : passive_tracking_settle_steps_first;
      if ((passive_ready_to_enter_docking ||
        passive_retry_second_entry_window ||
        passive_retry_force_second_entry ||
        passive_precision_reentry_window ||
        passive_force_first_entry_window) ||
        (!passive_target_mode_ && distance < tracking_distance_))
      {
        if (!passive_target_mode_ || tracking_hold_counter_ > passive_tracking_settle_steps) {
          current_phase_ = DockingPhase::DOCKING;
          tracking_hold_counter_ = 0;
          passive_docking_entry_count_++;
    if (passive_target_mode_) {
            if (passive_docking_entry_count_ == 1) {
              passive_retry_first_entry_lateral_abs_ = std::abs(terminal_lateral_error);
            }
            passive_retry_pending_second_entry_ = false;
          }
          min_docking_distance_ = distance;
        }
      } else if (
        (passive_target_mode_ && !passive_tracking_hold_window &&
        tracking_horizontal_distance > std::max(approach_distance_ * 1.85, 45.0)) ||
        (!passive_target_mode_ && distance > approach_distance_ * 1.2))
      {
        rendezvous_corridor_initialized_ = false;
        corridor_release_score_filtered_ = 0.0;
        corridor_release_armed_ = false;
        corridor_release_accept_counter_ = 0;
        tracking_hold_counter_ = 0;
        current_phase_ = DockingPhase::APPROACH;
      }
      break;
    }

    case DockingPhase::DOCKING:
      tracking_hold_counter_ = 0;
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
        soft_capture_envelope ||
        passive_soft_attach_envelope ||
        passive_corridor_capture_envelope)
      {
        current_phase_ = DockingPhase::COMPLETED;
      } else {
        const bool passive_retry_from_corridor_stall =
          passive_target_mode_ &&
          !passive_retry_used_ &&
          passive_docking_entry_count_ <= 1 &&
          rendezvous_corridor_initialized_ &&
          docking_hold_counter_ > passive_docking_retry_hold_steps &&
          distance > min_docking_distance_ + 0.45 &&
          std::abs(terminal_lateral_error) > 1.4 &&
          corridor_release_score_filtered_ < 0.22 &&
          relative_speed > 0.85;
        const bool passive_retry_after_early_release =
          passive_target_mode_ &&
          !passive_retry_used_ &&
          passive_docking_entry_count_ <= 1 &&
          !rendezvous_corridor_initialized_ &&
          docking_hold_counter_ > passive_docking_retry_hold_steps &&
          docking_hold_counter_ < passive_docking_retry_window_steps &&
          distance > min_docking_distance_ + 0.30 &&
          std::abs(terminal_lateral_error) > 1.8 &&
          std::abs(terminal_along_error) < 2.8 &&
          relative_speed > 0.95;
        const bool passive_retry_from_first_entry_stall =
          passive_target_mode_ &&
          !passive_retry_used_ &&
          passive_docking_entry_count_ == 1 &&
          docking_hold_counter_ > passive_docking_retry_hold_steps &&
          std::isfinite(passive_retry_first_entry_lateral_abs_) &&
          distance > min_docking_distance_ + 0.20 &&
          std::abs(terminal_lateral_error) >
          std::max(0.75, 0.35 * passive_retry_first_entry_lateral_abs_) &&
          std::abs(terminal_along_error) > 2.2 &&
          relative_speed > 0.50;
        if (
          passive_retry_from_corridor_stall ||
          passive_retry_after_early_release ||
          passive_retry_from_first_entry_stall)
        {
          passive_retry_used_ = true;
          passive_retry_pending_second_entry_ = true;
          passive_retry_tracking_best_lateral_abs_ = std::numeric_limits<double>::infinity();
          rendezvous_corridor_initialized_ = false;
          corridor_release_score_filtered_ = 0.0;
          corridor_release_armed_ = false;
          corridor_release_accept_counter_ = 0;
          current_phase_ = DockingPhase::TRACKING;
          tracking_hold_counter_ = 0;
          docking_hold_counter_ = 0;
          completion_hold_counter_ = 0;
          capture_hold_counter_ = 0;
        } else if (
          passive_target_mode_ &&
          rendezvous_corridor_initialized_ &&
          docking_hold_counter_ > passive_docking_retry_hold_steps &&
          distance > min_docking_distance_ + 0.6 &&
          std::abs(terminal_lateral_error) > 1.6 &&
          corridor_release_score_filtered_ < 0.16 &&
          relative_speed > 1.0)
        {
          passive_retry_used_ = true;
          passive_retry_pending_second_entry_ = true;
          passive_retry_tracking_best_lateral_abs_ = std::numeric_limits<double>::infinity();
          rendezvous_corridor_initialized_ = false;
          corridor_release_score_filtered_ = 0.0;
          corridor_release_armed_ = false;
          corridor_release_accept_counter_ = 0;
          current_phase_ = DockingPhase::TRACKING;
          tracking_hold_counter_ = 0;
          docking_hold_counter_ = 0;
          completion_hold_counter_ = 0;
          capture_hold_counter_ = 0;
        } else if (
          ((passive_target_mode_ &&
          docking_hold_counter_ > passive_docking_release_hold_steps &&
          distance > docking_release_distance &&
          distance > min_docking_distance_ + 16.0) ||
          (!passive_target_mode_ &&
          docking_hold_counter_ > static_cast<int>(control_rate_ * 0.8) &&
          distance > docking_release_distance &&
          distance > min_docking_distance_ + 6.0)))
        {
          rendezvous_corridor_initialized_ = false;
          corridor_release_score_filtered_ = 0.0;
          corridor_release_armed_ = false;
          corridor_release_accept_counter_ = 0;
          current_phase_ = DockingPhase::TRACKING;
          tracking_hold_counter_ = 0;
          docking_hold_counter_ = 0;
          completion_hold_counter_ = 0;
          capture_hold_counter_ = 0;
        }
      }
      break;

    default:
      break;
  }

  controller_debug_[0] = phaseToCode(current_phase_);
  controller_debug_[1] = tracking_horizontal_distance;
  controller_debug_[2] = tracking_along_error;
  controller_debug_[3] = tracking_lateral_error;
  controller_debug_[4] = along_relative_speed;
  controller_debug_[5] = lateral_relative_speed;
  controller_debug_[6] = docking_error;
  controller_debug_[7] = terminal_along_error;
  controller_debug_[8] = terminal_lateral_error;
  controller_debug_[9] = terminal_error.z();
  controller_debug_[10] = horizontalOnly(target_velocity).dot(track_direction);
  controller_debug_[11] = horizontalOnly(target_velocity).dot(lateral_direction);
  controller_debug_[12] = relative_speed;
  controller_debug_[13] = min_docking_distance_;
  controller_debug_[14] = rendezvous_corridor_initialized_ ? 1.0 : 0.0;
  controller_debug_[15] = passive_target_mode_ ? 1.0 : 0.0;
  controller_debug_[16] = corridor_release_score_filtered_;
  controller_debug_[17] = corridor_release_armed_ ? 1.0 : 0.0;
  controller_debug_[18] = static_cast<double>(corridor_release_accept_counter_);
  controller_debug_[19] = static_cast<double>(completion_hold_counter_);
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

void DockingController::computeCorridorPlan()
{
	// Time-parameterized trajectory: P(t) = C + (t / t_mini) * (T - C)
	// The carrier follows this trajectory from start position to the tangent
	// point T on the mini's orbit, arriving exactly when the mini does.
	// No hold, no waiting, no speed clamping — the trajectory IS the plan.

	const Eigen::Vector3d carrier_pos = poseToEigen(carrier_pose_);
	const Eigen::Vector3d mini_pos = poseToEigen(mini_pose_);

	if (mini_orbit_radius_ < 0.01 || mini_orbit_speed_ < 0.01) {
		corridor_plan_valid_ = false;
		return;
	}

	const double R = mini_orbit_radius_;
	const double orbit_speed = mini_orbit_speed_;
	const double omega = orbit_speed / R;
	const Eigen::Vector2d O(mini_orbit_center_.x(), mini_orbit_center_.y());
	const Eigen::Vector2d C(carrier_pos.x(), carrier_pos.y());
	const Eigen::Vector2d M(mini_pos.x(), mini_pos.y());

	const Eigen::Vector2d OC = C - O;
	const double d_oc = OC.norm();

	if (d_oc <= R * 1.001) {
		corridor_plan_valid_ = false;
		return;
	}

	// Geometric tangent points from C to orbit circle
	const double alpha = std::atan2(OC.y(), OC.x());
	const double beta = std::asin(R / d_oc);

	const auto tang_dir = [](double theta) -> Eigen::Vector2d {
		return Eigen::Vector2d(-std::sin(theta), std::cos(theta));
	};

	const double theta_1 = alpha + beta;
	const double theta_2 = alpha - beta;
	const Eigen::Vector2d T1 = O + R * Eigen::Vector2d(std::cos(theta_1), std::sin(theta_1));
	const Eigen::Vector2d T2 = O + R * Eigen::Vector2d(std::cos(theta_2), std::sin(theta_2));
	const double score_1 = (T1 - C).normalized().dot(tang_dir(theta_1));
	const double score_2 = (T2 - C).normalized().dot(tang_dir(theta_2));

	double theta_T = score_1 >= score_2 ? theta_1 : theta_2;
	Eigen::Vector2d T = score_1 >= score_2 ? T1 : T2;
	Eigen::Vector2d tang = tang_dir(theta_T);

	// Mini timing: use configured start phase (stable), not current position
	// (current position is unreliable during PX4 takeoff)
	const double theta_m = mini_orbit_start_phase_deg_ * M_PI / 180.0;
	double delta_theta = theta_T - theta_m;
	if (delta_theta < 0.0) { delta_theta += 2.0 * M_PI; }
	const double t_mini = delta_theta / omega;

	// Circular arc trajectory: from C to T, tangent to mini's orbit at T.
	// Arc center M_arc lies on the line O→T. Radius r satisfies |C-M|=|T-M|=r.
	const Eigen::Vector2d u = (T - O).normalized();  // O→T unit vector
	const double d = d_oc;
	const double cos_alpha = OC.dot(T - O) / (d * R);
	const double numer = d * d - R * R;
	const double denom = 2.0 * R * (d * cos_alpha - R);
	const double k = (std::abs(denom) > 0.01) ? (numer / denom) : 2.0;
	const Eigen::Vector2d M_arc = O + k * (T - O);
	const double r_arc = std::abs(k - 1.0) * R;

	// Arc angles from M_arc to C and T
	const double phi_start = std::atan2(C.y() - M_arc.y(), C.x() - M_arc.x());
	const double phi_end   = std::atan2(T.y() - M_arc.y(), T.x() - M_arc.x());
	// Shortest angular difference (signed)
	double dphi = phi_end - phi_start;
	while (dphi >  M_PI) { dphi -= 2.0 * M_PI; }
	while (dphi < -M_PI) { dphi += 2.0 * M_PI; }
	const double arc_len = r_arc * std::abs(dphi);

	// Target Z: idle hover altitude (30m), must match mini orbit altitude
	const double target_z = idle_hover_altitude_;
	corridor_traj_start_ = Eigen::Vector3d(C.x(), C.y(), carrier_pos.z());
	corridor_traj_end_   = Eigen::Vector3d(T.x(), T.y(), target_z);
	corridor_traj_start_time_ = 0.0;
	corridor_traj_active_ = false;

	// Store arc parameters for trajectory tracking
	corridor_arc_center_ = M_arc;
	corridor_arc_radius_ = r_arc;
	corridor_arc_phi_start_ = phi_start;
	corridor_arc_delta_phi_ = dphi;

	// Set arc speed to a natural value (not t_mini-matched).
	// Carrier flies arc at consistent speed, then continues on tangent.
	// Mini catches up from behind at its natural speed.
	// Arc at 8 m/s. Hold at launch for timing: hold = t_mini - t_arc
	const double arc_speed = 8.0;
	corridor_traj_duration_ = arc_len / arc_speed;
	corridor_planned_speed_ = arc_speed;
	corridor_mini_arrival_delay_ = t_mini;
	// Hold at launch so arc completes when mini reaches T
	corridor_hold_duration_ = std::max(0.0, t_mini - corridor_traj_duration_);

	double trigger_phase = theta_T;
	while (trigger_phase < 0.0) { trigger_phase += 2.0 * M_PI; }
	while (trigger_phase >= 2.0 * M_PI) { trigger_phase -= 2.0 * M_PI; }

	corridor_tangent_point_ = Eigen::Vector3d(T.x(), T.y(), idle_hover_altitude_);
	corridor_carrier_target_ = corridor_tangent_point_;
	corridor_tangent_dir_ = Eigen::Vector3d(tang.x(), tang.y(), 0.0);
	corridor_plan_valid_ = true;
}

void DockingController::approachPhaseControl(geometry_msgs::msg::Twist& carrier_cmd,
                                            geometry_msgs::msg::Twist& mini_cmd)
{
  const Eigen::Vector3d relative_pos = relative_estimator_->getRelativePosition();
  const Eigen::Vector3d relative_error = relative_pos - desired_relative_position_;
  const Eigen::Vector3d relative_vel = relative_estimator_->getRelativeVelocity();
  const Eigen::Vector3d target_velocity(
    mini_twist_.linear.x, mini_twist_.linear.y, mini_twist_.linear.z);
  const Eigen::Vector3d carrier_velocity = carrierVelocity(carrier_twist_);
  const Eigen::Vector3d los_direction = computeLosDirection(-horizontalOnly(relative_error), target_velocity);
  const Eigen::Vector3d track_direction =
    computeLosDirection(horizontalOnly(target_velocity), -horizontalOnly(relative_error));
  const Eigen::Vector3d lateral_direction(-track_direction.y(), track_direction.x(), 0.0);
  const double distance = relative_error.norm();
  const double lateral_error = std::abs(relative_error.y());
  const double vertical_error = relative_error.z();
  const double track_lateral_error = horizontalOnly(relative_error).dot(lateral_direction);
  const double track_along_error = horizontalOnly(relative_error).dot(track_direction);
  const double track_lateral_speed = horizontalOnly(relative_vel).dot(lateral_direction);
  const double track_along_speed = horizontalOnly(relative_vel).dot(track_direction);

  if (passive_target_mode_) {
    // Departure hold: carrier waits at idle position before starting approach.
    // Hold duration is computed by computeCorridorPlan() — a deterministic rule
    // based on tangent-intercept geometry, not a guessed fraction.
    const Eigen::Vector3d carrier_pos = poseToEigen(carrier_pose_);
    if (!departure_hold_completed_) {
      if (!departure_hold_active_) {
        departure_hold_active_ = true;
        idle_carrier_hold_position_ = carrier_pos;
        computeCorridorPlan();
      }
      ++approach_hold_counter_;
      const double hold_duration = corridor_hold_duration_;
      const double hold_time = approach_hold_counter_ / control_rate_;
      if (hold_time < hold_duration) {
        carrier_velocity_command_.setZero();
        carrier_position_setpoint_ = idle_carrier_hold_position_;
        mini_velocity_command_.setZero();
        mini_position_setpoint_ = poseToEigen(mini_pose_);
        carrier_cmd.linear.x = 0.0;
        carrier_cmd.linear.y = 0.0;
        carrier_cmd.linear.z = 0.0;
        carrier_cmd.angular.z = 0.0;
        mini_cmd.linear.x = 0.0;
        mini_cmd.linear.y = 0.0;
        mini_cmd.linear.z = 0.0;
        mini_cmd.angular.z = 0.0;
        return;
      }
      departure_hold_completed_ = true;
      // Start the time-parameterized trajectory immediately (hold=0)
      if (corridor_plan_valid_ && !corridor_traj_active_) {
        corridor_traj_counter_ = 0;
        corridor_traj_active_ = true;
      }
    }

    // ── Corridor Plan Trajectory Tracking (circular arc) ──
    // Arc from C to T, tangent to mini's orbit at T.
    // P(φ) = M_arc + r * (cos φ, sin φ), φ = φ_start + frac * Δφ
    if (corridor_plan_valid_ && corridor_traj_active_) {
      ++corridor_traj_counter_;
      const double elapsed = corridor_traj_counter_ / control_rate_;
      const double frac = clampValue(elapsed / std::max(corridor_traj_duration_, 0.1), 0.0, 1.0);
      const double phi = corridor_arc_phi_start_ + frac * corridor_arc_delta_phi_;
      const double c = std::cos(phi);
      const double s = std::sin(phi);
      const Eigen::Vector2d pos_2d =
        corridor_arc_center_ + corridor_arc_radius_ * Eigen::Vector2d(c, s);
      // Tangent direction along the arc (derivative of position wrt φ, normalized)
      const Eigen::Vector2d tang_2d =
        (corridor_arc_delta_phi_ > 0.0)
          ? Eigen::Vector2d(-s, c)
          : Eigen::Vector2d(s, -c);
      const double z_start = corridor_traj_start_.z();
      const double z_end   = corridor_traj_end_.z();
      const Eigen::Vector3d traj_pos(pos_2d.x(), pos_2d.y(),
                                      z_start + frac * (z_end - z_start));
      // Z correction during arc: pull carrier toward mini's altitude
      const double z_nominal_vel = (z_end - z_start) / std::max(corridor_traj_duration_, 0.1);
      const double z_correction = 0.4 * (relative_pos.z() - 0.3);
      const Eigen::Vector3d traj_vel(
        tang_2d.x() * corridor_planned_speed_,
        tang_2d.y() * corridor_planned_speed_,
        z_nominal_vel + z_correction);

      carrier_position_setpoint_ = traj_pos;
      carrier_velocity_command_ = traj_vel;
      mini_velocity_command_.setZero();
      mini_position_setpoint_ = poseToEigen(mini_pose_);

      // Phase 2: two-stage terminal docking.
      // Stage 1 (gap > 10m): step speed for coarse closure.
      // Stage 2 (gap ≤ 10m): PD controller for fine gap + Z constraint.
      // Target: signed_gap = -0.5m (carrier 0.5m ahead), mini 0.3m above carrier.
      if (frac >= 0.99) {
        const double phase2_t = elapsed - corridor_traj_duration_;
        const double signed_gap = relative_pos.dot(corridor_tangent_dir_);
        const double abs_gap = std::abs(signed_gap);
        const double rel_z = relative_pos.z();  // >0 means mini above carrier
        const double rel_vel_along = relative_vel.dot(corridor_tangent_dir_);
        double speed;
        double vz = 0.0;

        if (abs_gap > 10.0) {
          // Stage 1: step speed for coarse gap closure
          speed = 7.5;
          if (signed_gap < -8.0)       speed = 6.5;
          else if (signed_gap < -3.0)  speed = 7.0;
          else if (signed_gap < -1.0)  speed = 7.3;
          else if (signed_gap > 1.0)   speed = 8.5;
          // Gentle Z correction during coarse phase too
          vz = 0.5 * (rel_z - 0.3);
          vz = clampValue(vz, -1.5, 1.5);
        } else {
          // Stage 2: PD control for fine terminal docking
          // Along-track: PD on signed gap, target carrier 0.5m ahead
          const double gap_error = signed_gap + 0.5;  // 0 when carrier 0.5m ahead
          speed = 8.0 - 0.3 * gap_error - 0.15 * rel_vel_along;
          speed = clampValue(speed, 7.0, 9.0);
          // Z: PD to keep mini 0.3m above carrier
          const double z_error = rel_z - 0.3;
          vz = 0.6 * z_error - 0.2 * relative_vel.z();
          vz = clampValue(vz, -1.5, 1.5);
        }

        carrier_velocity_command_ =
          corridor_tangent_dir_ * speed + Eigen::Vector3d(0.0, 0.0, vz);
        carrier_position_setpoint_ =
          corridor_traj_end_ + carrier_velocity_command_ * phase2_t;

        // Complete: gap < 0.25m, carrier ahead, mini above carrier 0.15-0.55m
        if (phase2_t > 15.0 ||
            (abs_gap < 0.25 && signed_gap < 0 && rel_z > 0.15 && rel_z < 0.55)) {
          corridor_plan_valid_ = false;
          current_phase_ = DockingPhase::COMPLETED;
        }
        carrier_cmd.linear.x = carrier_velocity_command_(0);
        carrier_cmd.linear.y = carrier_velocity_command_(1);
        carrier_cmd.linear.z = carrier_velocity_command_(2);
        carrier_cmd.angular.z = 0.0;
        mini_cmd.linear.x = 0.0;
        mini_cmd.linear.y = 0.0;
        mini_cmd.linear.z = 0.0;
        mini_cmd.angular.z = 0.0;
        return;
      }

      carrier_cmd.linear.x = carrier_velocity_command_(0);
      carrier_cmd.linear.y = carrier_velocity_command_(1);
      carrier_cmd.linear.z = carrier_velocity_command_(2);
      carrier_cmd.angular.z = 0.0;
      mini_cmd.linear.x = 0.0;
      mini_cmd.linear.y = 0.0;
      mini_cmd.linear.z = 0.0;
      mini_cmd.angular.z = 0.0;
      return;
    }

    // Approach: explicit LOS/intercept guidance to pull lateral geometry in before close tracking.
    const double terminal_z_band_min = 0.25;
    const double terminal_z_band_max = 0.95;
    const double z_out_of_band =
      relative_pos.z() < terminal_z_band_min
      ? terminal_z_band_min - relative_pos.z()
      : (relative_pos.z() > terminal_z_band_max
        ? relative_pos.z() - terminal_z_band_max
        : 0.0);
    const double z_guard_blend = clampValue(z_out_of_band / 0.60, 0.0, 1.0);
    double approach_speed_cap =
      (1.0 - z_guard_blend) * carrier_approach_speed_limit_ +
      z_guard_blend * std::min(0.45 * carrier_approach_speed_limit_, 4.0);
    const bool z_guard_freeze_along =
      z_out_of_band > 0.6 && distance < 12.0;
    const double pursuit_speed =
      clampValue(
        1.0 + 0.11 * distance + 0.05 * lateral_error,
        1.4,
        approach_speed_cap);
    const double metric_distance = relative_pos.norm();
    // When corridor plan is active, track the planned speed instead of
    // using reactive pursuit. The plan already accounts for timing.
    double closing_speed = corridor_plan_valid_
      ? corridor_planned_speed_
      : pursuit_speed;
    // Skip standoff when corridor plan is active — planned speed handles timing.
    if (!corridor_plan_valid_ && z_out_of_band > 0.35) {
      const double standoff_distance =
        10.5 + 4.0 * clampValue((z_out_of_band - 0.35) / 1.10, 0.0, 1.0);
      if (metric_distance < standoff_distance) {
        const double backoff_blend =
          clampValue((standoff_distance - metric_distance) / 1.6, 0.0, 1.0);
        closing_speed =
          (1.0 - backoff_blend) * closing_speed -
          backoff_blend * 1.0;
      }
    }
    // When corridor plan is valid, use the fixed tangent direction for
    // macroscopic guidance — carrier flies toward the geometric tangent point
    // instead of chasing the moving mini. This eliminates velocity reversal.
    Eigen::Vector3d approach_direction;
    if (corridor_plan_valid_) {
      // Fly straight toward the ahead target on the tangent (carrier leads).
      const Eigen::Vector3d to_target = corridor_carrier_target_ - carrier_pos;
      approach_direction = to_target.norm() > 0.01
        ? to_target.normalized()
        : corridor_tangent_dir_;
    } else {
      approach_direction = los_direction;
    }
    // Invalidate corridor plan when mini is within transition range
    if (corridor_plan_valid_ && relative_pos.norm() < approach_distance_ * 0.8) {
      corridor_plan_valid_ = false;
    }
    const Eigen::Vector3d closing_velocity = approach_direction * closing_speed;
    Eigen::Vector3d intercept_velocity = closing_velocity;
    // Approach direction locking: only needed when corridor plan is not active.
    // With a valid corridor plan, the direction is already fixed geometrically.
    if (!corridor_plan_valid_ && departure_hold_completed_ && carrier_approach_speed_limit_ > 0.1) {
      const Eigen::Vector3d h_cmd = horizontalOnly(intercept_velocity);
      const double h_speed = h_cmd.norm();
      if (!approach_direction_locked_ && h_speed > 0.5) {
        primary_approach_tangent_ = h_cmd.normalized();
        approach_direction_locked_ = true;
      }
      if (approach_direction_locked_ && h_speed > 0.5) {
        const Eigen::Vector3d h_dir = h_cmd / h_speed;
        const double dot = h_dir.dot(primary_approach_tangent_);
        if (dot < 0.259) {
          const double angle = std::acos(std::max(-1.0, std::min(1.0, dot)));
          const double target_angle = 75.0 * M_PI / 180.0;
          const double cross = h_dir.x() * primary_approach_tangent_.y()
                            - h_dir.y() * primary_approach_tangent_.x();
          const double rot = (cross > 0.0 ? 1.0 : -1.0) * (angle - target_angle);
          const double c = std::cos(rot);
          const double s = std::sin(rot);
          intercept_velocity.x() = (h_dir.x() * c - h_dir.y() * s) * h_speed;
          intercept_velocity.y() = (h_dir.x() * s + h_dir.y() * c) * h_speed;
        }
      }
    }
    const double vertical_intercept_gain = 0.90 + 0.70 * z_guard_blend;
    const double vertical_intercept_limit = 1.4 + 1.2 * z_guard_blend;
    intercept_velocity.z() =
      target_velocity.z() +
      clampValue(vertical_error * vertical_intercept_gain,
      -vertical_intercept_limit,
      vertical_intercept_limit);

    const double preview_horizon =
      clampValue(intercept_lookahead_ * 0.7 + 0.025 * distance, 0.8, 1.8);
    // When corridor plan is valid, intercept_position is the fixed tangent point.
    // Otherwise, fall back to chasing the mini with a preview horizon.
    Eigen::Vector3d intercept_position;
    if (corridor_plan_valid_) {
      intercept_position = corridor_carrier_target_ - desired_relative_position_;
    } else {
      intercept_position = poseToEigen(mini_pose_) + target_velocity * preview_horizon
                         - desired_relative_position_;
    }
	    const Eigen::Vector3d direct_position =
	      poseToEigen(mini_pose_) - desired_relative_position_;

	    const double close_blend = clampValue((18.0 - distance) / 18.0, 0.0, 1.0);
	    const double lateral_correction_gain = 0.40 + 0.68 * close_blend;
	    const double lateral_damping_gain = 0.20 + 0.34 * close_blend;
	    const double along_gain_scale = z_guard_freeze_along ? 0.0 : 1.0;
    const double along_correction_gain = (0.06 + 0.10 * close_blend) * along_gain_scale;
    const double along_damping_gain = (0.04 + 0.08 * close_blend) * along_gain_scale;
    const Eigen::Vector3d velocity_correction =
      track_direction * clampValue(
        -along_correction_gain * track_along_error - along_damping_gain * track_along_speed,
        -0.6,
        0.6) +
      lateral_direction * clampValue(
        +lateral_correction_gain * track_lateral_error + lateral_damping_gain * track_lateral_speed,
        -3.2,
        3.2) +
      Eigen::Vector3d(
        0.0,
        0.0,
        clampValue(
          (0.74 + 0.40 * z_guard_blend) * vertical_error - 0.24 * relative_vel.z(),
          -(1.60 + 1.00 * z_guard_blend),
          +(1.60 + 1.00 * z_guard_blend)));
    // When corridor plan is active, suppress velocity correction (PD) so the
    // carrier tracks the planned speed exactly. Corrections would add extra
    // speed and make the carrier arrive early.
    Eigen::Vector3d desired_velocity = corridor_plan_valid_
      ? intercept_velocity
      : (intercept_velocity + velocity_correction);
    if (z_guard_freeze_along) {
      const double extra_along =
        (closing_velocity + velocity_correction).dot(track_direction);
      desired_velocity -= track_direction * extra_along;
    }
	    carrier_velocity_command_ =
	      clampNorm(desired_velocity - 0.08 * (carrier_velocity - target_velocity),
	      carrier_approach_speed_limit_);
	    // When corridor plan is active, enforce planned speed so the
	    // carrier doesn't arrive at T early and have to wait.
	    if (corridor_plan_valid_) {
	      const double planned = corridor_planned_speed_;
	      const double h_norm = horizontalOnly(carrier_velocity_command_).norm();
	      if (h_norm > planned * 1.05) {
	        const Eigen::Vector3d h_dir = horizontalOnly(carrier_velocity_command_).normalized();
	        carrier_velocity_command_.x() = h_dir.x() * planned;
	        carrier_velocity_command_.y() = h_dir.y() * planned;
	      }
	    }
	    // Vertical dynamics dominate the z-band dwell time inside the <=10m window. Clamp the
	    // vertical command in close range so we don't dive through the [0.25, 0.95] band too fast.
	    if (metric_distance < 15.0) {
	      const double rel_z = relative_pos.z();
	      double z_speed_limit = 1.60;
	      // If the mini is moving vertically (e.g. gliding), allow the carrier to match that
	      // motion. Otherwise, a strict clamp can make rel_z traverse the evaluation band too fast
	      // simply because the carrier cannot follow the target's vertical speed.
	      const double target_vz_abs = std::abs(target_velocity.z());
	      const bool target_vertical_active = target_vz_abs > 0.75;
	      if (rel_z < 1.60) {
	        z_speed_limit = std::min(z_speed_limit, 0.90);
	      }
	      if (!target_vertical_active && rel_z > 0.10 && rel_z < 1.10) {
	        z_speed_limit = std::min(z_speed_limit, 0.55);
	      }
	      if (target_vertical_active) {
	        z_speed_limit = std::max(z_speed_limit, target_vz_abs + 0.55);
	      }
	      carrier_velocity_command_.z() =
	        clampValue(carrier_velocity_command_.z(), -z_speed_limit, z_speed_limit);
	    }
	    mini_velocity_command_.setZero();
	    // When cross-track is still large, bias the position setpoint toward "directly under mini"
	    // (remove lead) to collapse lateral geometry more reliably before entering TRACKING/DOCKING.
	    const double pull_in_dist_blend = clampValue((20.0 - distance) / 10.0, 0.0, 1.0);
	    const double pull_in_lat_blend = clampValue((std::abs(track_lateral_error) - 0.6) / 1.2, 0.0, 1.0);
	    const double pull_in_blend = pull_in_dist_blend * pull_in_lat_blend;
	    carrier_position_setpoint_ =
	      (1.0 - pull_in_blend) * intercept_position +
	      pull_in_blend * direct_position;
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
    // Tracking: lock onto a rendezvous corridor and chase a point on that corridor, not the aircraft body.
    const Eigen::Vector3d track_direction =
      computeLosDirection(horizontalOnly(target_velocity), rendezvous_corridor_axis_);
    if (!rendezvous_corridor_initialized_) {
      rendezvous_corridor_initialized_ = true;
      corridor_release_score_filtered_ = 0.0;
      corridor_release_armed_ = false;
      corridor_release_accept_counter_ = 0;
    }
    // Keep corridor geometry consistent even while the mini tangent direction evolves by
    // re-anchoring the corridor at the current mini position every tick.
    rendezvous_corridor_axis_ = track_direction;
    const Eigen::Vector3d mini_position = poseToEigen(mini_pose_);
    rendezvous_corridor_anchor_ = mini_position;
    const Eigen::Vector3d lateral_direction(-track_direction.y(), track_direction.x(), 0.0);
    const double along_error = horizontalOnly(relative_error).dot(track_direction);
    const double lateral_error = horizontalOnly(relative_error).dot(lateral_direction);
    const Eigen::Vector3d terminal_error = relative_pos - terminal_relative_position_;
    const double terminal_along_error = horizontalOnly(terminal_error).dot(track_direction);
    const double metric_distance = relative_pos.norm();
    const Eigen::Vector3d carrier_position = poseToEigen(carrier_pose_);
    const double carrier_along_speed = horizontalOnly(carrier_velocity).dot(track_direction);
    const double carrier_lateral_speed = horizontalOnly(carrier_velocity).dot(lateral_direction);
    const double target_along_speed = horizontalOnly(target_velocity).dot(track_direction);
    const double target_lateral_speed = horizontalOnly(target_velocity).dot(lateral_direction);
    const double rendezvous_along =
      clampValue(3.0 + 0.28 * std::abs(along_error), 2.8, 5.5);
    const double rendezvous_lateral =
      clampValue(0.12 * lateral_error, -1.8, 1.8);
    Eigen::Vector3d rendezvous_point =
      mini_position +
      track_direction * rendezvous_along +
      lateral_direction * rendezvous_lateral;
    rendezvous_point.z() = mini_position.z();
    const Eigen::Vector3d intercept_position =
      rendezvous_point - desired_relative_position_;
    const Eigen::Vector3d direct_position =
      poseToEigen(mini_pose_) - desired_relative_position_;
    const Eigen::Vector3d corridor_position_error = intercept_position - carrier_position;
    const double corridor_along_error = horizontalOnly(corridor_position_error).dot(track_direction);
    const double corridor_lateral_error = horizontalOnly(corridor_position_error).dot(lateral_direction);
    const double lateral_relative_speed = horizontalOnly(relative_vel).dot(lateral_direction);
    // Start tapering in "close-range" behavior earlier so we have time to shed large cross-track
    // errors before entering the <=10m metric window.
    const double tracking_close_blend = clampValue((18.0 - distance) / 12.0, 0.0, 1.0);
    const Eigen::Vector3d forward_intercept =
      track_direction * clampValue(
        0.30 * corridor_along_error -
        0.24 * (carrier_along_speed - target_along_speed) +
        0.04 * along_error,
        -0.6,
        2.0);
    const bool tracking_lat_strong =
      passive_target_mode_ && distance < 22.0 && std::abs(lateral_error) > 1.0;
    const double lateral_intercept_limit =
      clampValue(
        1.00 + 0.45 * std::abs(lateral_error),
        1.00,
        tracking_lat_strong ? 3.60 : 3.00);
    const double tracking_lateral_priority =
      (std::abs(lateral_error) > 0.7 && distance < 22.0) ? 1.0 : 0.0;
    const bool tracking_hard_lateral =
      std::abs(lateral_error) > 2.2 && distance < 12.0;
    const double lateral_intercept_gain =
      1.0 +
      0.35 * tracking_lateral_priority +
      (tracking_hard_lateral ? 0.60 : 0.0) +
      (tracking_lat_strong ? 0.35 : 0.0);
    const Eigen::Vector3d lateral_intercept =
      lateral_direction * clampValue(
        -lateral_intercept_gain *
        ((0.38 + 0.25 * tracking_close_blend) * corridor_lateral_error +
        (0.34 + 0.20 * tracking_close_blend) * (carrier_lateral_speed - target_lateral_speed) +
        (0.08 + 0.10 * tracking_close_blend) * lateral_error),
        -lateral_intercept_limit,
        lateral_intercept_limit);
    const double tracking_z_band_min = 0.25;
    const double tracking_z_band_max = 0.95;
    const bool tracking_z_in_band =
      relative_pos.z() > tracking_z_band_min &&
      relative_pos.z() < tracking_z_band_max;
    const bool tracking_lat_hold_active = tracking_lat_hold_counter_ > 0;
	    const bool tracking_lat_settle =
	      passive_target_mode_ &&
	      tracking_z_in_band &&
	      distance < 10.0 &&
	      std::abs(lateral_error) < 0.35;
	    const bool tracking_lat_fine =
	      passive_target_mode_ &&
	      tracking_z_in_band &&
	      distance < 10.0 &&
	      std::abs(lateral_error) < 0.25;
	    const bool tracking_lat_mid =
	      passive_target_mode_ &&
	      tracking_z_in_band &&
	      distance < 12.0 &&
	      std::abs(lateral_error) > 0.25;
    const int tracking_z_recovery_steps = std::max(1, static_cast<int>(control_rate_ * 0.20));
    const bool tracking_z_recovery_active =
      passive_target_mode_ && terminal_z_out_counter_ >= tracking_z_recovery_steps;
    double vertical_gain = 0.42;
    double vertical_limit = 0.6;
    double vertical_match_gain = 0.20;
    if (tracking_z_recovery_active) {
      vertical_gain = 0.70;
      vertical_limit = 0.9;
    }
    if (tracking_lat_hold_active) {
      vertical_gain = std::max(vertical_gain, 0.55);
      vertical_limit = std::min(vertical_limit, 0.50);
    }
    if (passive_target_mode_ && tracking_close_blend > 0.25) {
      const double abs_target_vz = std::abs(target_velocity.z());
      const double abs_rel_vz = std::abs(relative_vel.z());
      const double match_boost =
        std::max(
          clampValue((abs_target_vz - 0.45) / 1.10, 0.0, 1.0),
          clampValue((abs_rel_vz - 1.10) / 1.30, 0.0, 1.0));
      vertical_match_gain = 0.20 + 0.30 * match_boost;
      vertical_limit = std::max(vertical_limit, 0.6 + 1.1 * match_boost);
      vertical_gain = std::max(vertical_gain, 0.42 + 0.18 * match_boost);
    }
    const double vertical_damping =
      tracking_lat_hold_active ? -0.35 * relative_vel.z() : 0.0;
    double vertical_cmd =
      clampValue(
        vertical_gain * corridor_position_error.z() -
        vertical_match_gain * (target_velocity.z() - carrier_velocity.z()) +
        vertical_damping +
        0.08 * relative_error.z(),
        -vertical_limit,
        vertical_limit);
    // Keep some margin from the z-band boundaries in the close-range regime; otherwise
    // small vertical oscillations can repeatedly drop us out of the band and zero-out the
    // `hold_lat_0p2_zband_sec` metric.
    if (passive_target_mode_ && tracking_close_blend > 0.20) {
      const double z_soft_min = 0.30;
      const double z_soft_max = 0.90;
      if (relative_pos.z() < z_soft_min) {
        vertical_cmd = std::min(vertical_cmd, 0.0);
      } else if (relative_pos.z() > z_soft_max) {
        vertical_cmd = std::max(vertical_cmd, 0.0);
      }
    }
    const Eigen::Vector3d vertical_intercept(0.0, 0.0, vertical_cmd);
    const Eigen::Vector3d desired_velocity =
      target_velocity +
      forward_intercept +
      lateral_intercept +
      vertical_intercept - 0.06 * guidance_cmd;
	    double desired_along_velocity =
	      clampValue(desired_velocity.dot(track_direction), 0.0, carrier_tracking_speed_limit_);
	    if (tracking_lat_hold_active) {
	      // In lat-hold we still need to match the mini's along-track speed; otherwise the
	      // mini will pull away and we drift out of the <=10m hold window even if lateral/z
	      // are good. Keep along velocity near target with a small delta limit.
	      const double along_relative_speed = target_along_speed - carrier_along_speed;
	      const double hold_along_delta_limit_base = tracking_lat_fine ? 0.55 : 0.85;
	      const double hold_near_window_blend =
	        clampValue((distance - 9.2) / 0.9, 0.0, 1.0);
	      const double hold_separating_blend =
	        clampValue((along_relative_speed - 0.20) / 1.20, 0.0, 1.0);
	      const double hold_along_delta_limit =
	        hold_along_delta_limit_base +
	        0.85 * hold_near_window_blend * hold_separating_blend;
	      // Never let the carrier fall notably behind along-track while the lat/z constraints
	      // are satisfied; otherwise `relative_distance` will exceed 10m before we accumulate
	      // the >=0.30s hold window.
	      const double hold_along_delta_below_limit = 0.20;
	      desired_along_velocity =
	        clampValue(
	          desired_along_velocity - target_along_speed,
	          -hold_along_delta_below_limit,
	          hold_along_delta_limit) + target_along_speed;
	      // If we are about to drift outside the <=10m evaluation window, bias slightly faster
	      // than the mini so we remain inside long enough to accumulate hold time.
	      const double hold_distance_bias =
	        0.90 *
	        clampValue((distance - 9.4) / 0.6, 0.0, 1.0) *
	        hold_separating_blend;
	      desired_along_velocity =
	        clampValue(desired_along_velocity + hold_distance_bias, 0.0, carrier_tracking_speed_limit_);
	      desired_along_velocity =
	        clampValue(desired_along_velocity, 0.0, carrier_tracking_speed_limit_);
	    }
	    if (passive_target_mode_ && tracking_z_in_band) {
	      // If we are inside the close-range regime but still far behind the terminal point,
	      // bias along-track speed above the mini so we don't "look aligned" laterally yet
	      // drift outside the <=10m metric window.
	      const double terminal_along_error_pos = std::max(0.0, terminal_along_error);
	      const double close_catchup_blend =
	        clampValue((18.0 - metric_distance) / 6.0, 0.0, 1.0) *
	        clampValue((1.60 - std::abs(lateral_error)) / 1.60, 0.0, 1.0) *
	        clampValue((terminal_along_error_pos - 3.2) / 6.0, 0.0, 1.0);
	      const double min_along_catchup =
	        clampValue(
	          target_along_speed + 3.20 * close_catchup_blend,
	          0.0,
	          carrier_tracking_speed_limit_);
	      desired_along_velocity = std::max(desired_along_velocity, min_along_catchup);
	    }
	    if (passive_target_mode_ && tracking_z_in_band) {
	      // In the metric window (<=10m), along error dominates relative_distance once lateral is small.
	      // If we do not actively close along error, we can momentarily satisfy |lat|<=0.2 but drift
	      // just outside 10m before accumulating the >=0.30s hold requirement.
	      const double distance_guard_blend =
	        clampValue((metric_distance - 8.8) / 1.4, 0.0, 1.0) *
	        clampValue((0.60 - std::abs(lateral_error)) / 0.60, 0.0, 1.0);
	      const double along_relative_speed = target_along_speed - carrier_along_speed;
	      const double catchup_active =
	        clampValue((along_relative_speed - 0.05) / 0.60, 0.0, 1.0) *
	        clampValue((std::max(0.0, terminal_along_error) - 2.0) / 6.0, 0.0, 1.0) *
	        distance_guard_blend;
	      const double catchup_boost = 3.20 * catchup_active;
	      const double min_along_guard =
	        clampValue(
	          target_along_speed + 0.45 * distance_guard_blend + catchup_boost,
	          0.0,
	          carrier_tracking_speed_limit_);
	      desired_along_velocity = std::max(desired_along_velocity, min_along_guard);
	    }
	    if (passive_target_mode_ && tracking_z_in_band && !tracking_lat_hold_active) {
	      // Outside the <=10m evaluation window, a large along error is very hard to recover from if
	      // we only command ~target speed. When lateral geometry is already acceptable, actively
	      // boost along speed (within the tracking limit) so end-of-run XY separation doesn't remain
	      // large after a DOCKING->TRACKING retry.
	      const double terminal_along_error_pos = std::max(0.0, terminal_along_error);
	      const double outside_window_blend =
	        clampValue((metric_distance - 10.0) / 6.0, 0.0, 1.0);
	      const double recover_blend =
	        outside_window_blend *
	        clampValue((0.90 - std::abs(lateral_error)) / 0.90, 0.0, 1.0) *
	        clampValue((terminal_along_error_pos - 8.0) / 10.0, 0.0, 1.0);
	      const double min_along_recover =
	        clampValue(
	          target_along_speed + 2.80 * recover_blend,
	          0.0,
	          carrier_tracking_speed_limit_);
	      desired_along_velocity = std::max(desired_along_velocity, min_along_recover);
	    }
	    if (passive_target_mode_) {
	      // Don't over-slow the along-axis when we are still significantly behind the mini.
	      // Otherwise, if target speed exceeds our along command for long stretches, cross-track
	      // can stagnate inside the <=10m window (hold stays 0).
	      const double lateral_slowdown_along_gate =
	        clampValue((2.8 - std::abs(terminal_along_error)) / 2.8, 0.0, 1.0);
	      const double lateral_slowdown_blend =
	        clampValue((std::abs(lateral_error) - 0.8) / 1.0, 0.0, 1.0) *
	        tracking_close_blend *
	        lateral_slowdown_along_gate;
	      desired_along_velocity =
	        std::min(
	          desired_along_velocity,
	          carrier_tracking_speed_limit_ * (1.0 - 0.60 * lateral_slowdown_blend));
	    }
    if (passive_target_mode_ && terminal_along_error > 0.5) {
      // When the carrier is behind, never command a notably slower along-track speed than the
      // mini; otherwise the along error grows quickly and we end the run with large XY separation.
      const double min_match_along =
        clampValue(target_along_speed - 0.10, 0.0, carrier_tracking_speed_limit_);
      desired_along_velocity = std::max(desired_along_velocity, min_match_along);
    }
    if (tracking_z_recovery_active) {
      // During z recovery we should avoid *additional* horizontal closure, but we must still match
      // the mini's along-track speed. A hard absolute cap can make the carrier fall behind and blow
      // up terminal XY separation before z is recovered.
      const double max_along_delta = 0.35;
      const double min_along_delta = -0.20;
      desired_along_velocity =
        clampValue(
          desired_along_velocity - target_along_speed,
          min_along_delta,
          max_along_delta) + target_along_speed;
      desired_along_velocity = clampValue(desired_along_velocity, 0.0, carrier_tracking_speed_limit_);
    }
    const double base_lateral_velocity_limit =
      clampValue(0.60 + 0.35 * std::abs(lateral_error), 0.60, 3.20);
    double lateral_velocity_limit =
      passive_target_mode_ ? 1.15 * base_lateral_velocity_limit : base_lateral_velocity_limit;
    if (tracking_lateral_priority > 0.5) {
      lateral_velocity_limit = std::min(1.70 * lateral_velocity_limit, 4.2);
    }
    if (tracking_hard_lateral) {
      lateral_velocity_limit = std::min(2.10 * lateral_velocity_limit, 6.2);
    }
    if (tracking_lat_strong) {
      lateral_velocity_limit = std::min(2.00 * lateral_velocity_limit, 6.0);
    }
    if (tracking_lat_fine) {
      lateral_velocity_limit = std::min(lateral_velocity_limit, 0.7);
    }
    if (tracking_lat_hold_active) {
      lateral_velocity_limit = std::min(lateral_velocity_limit, 0.45);
    }
    // PD on terminal cross-track error in the mini tangent frame.
    // relative_velocity = mini - carrier, so a positive lateral_error should command
    // a positive carrier lateral speed (to make relative lateral speed negative).
    const double lateral_pd_scale =
      passive_target_mode_
      ? (tracking_lat_settle ? 1.45 : 1.15) *
        (tracking_lat_hold_active ? 1.15 : 1.0) *
        (tracking_hard_lateral ? 1.55 : 1.0) *
        (tracking_lat_strong ? 1.35 : 1.0)
      : 1.0;
    double lateral_p_gain = 1.02 * lateral_pd_scale;
    double lateral_d_gain = 0.44 * lateral_pd_scale;
    if (tracking_lat_mid) {
      lateral_p_gain *= 1.40;
      lateral_d_gain *= 1.25;
    }
    if (tracking_lat_strong) {
      lateral_p_gain *= 1.25;
      lateral_d_gain *= 1.10;
    }
    if (tracking_lat_fine) {
      lateral_p_gain *= 0.65;
      lateral_d_gain *= 2.20;
    }
    if (tracking_lat_hold_active) {
      lateral_p_gain *= 0.75;
      lateral_d_gain *= 1.25;
    }
    const double lateral_pd =
      lateral_p_gain * lateral_error +
      lateral_d_gain * lateral_relative_speed;
    double desired_lateral_velocity =
      clampValue(target_lateral_speed + lateral_pd, -lateral_velocity_limit, lateral_velocity_limit);
    if (tracking_lat_hold_active) {
      const double hold_lateral_limit = 0.12;
      desired_lateral_velocity =
        clampValue(desired_lateral_velocity, -hold_lateral_limit, hold_lateral_limit);
    }
    if (passive_target_mode_ && tracking_z_in_band) {
      // Acceleration limiting makes it hard to recover if we allow along-track speed to drop
      // well below the mini while already near the <=10m window. Reserve enough along budget
      // so the carrier does not fall behind and drift out before the hold timer can accumulate.
      const double terminal_along_error_pos = std::max(0.0, terminal_along_error);
      const double along_floor_blend =
        clampValue((16.5 - metric_distance) / 6.5, 0.0, 1.0) *
        clampValue((terminal_along_error_pos - 3.0) / 6.0, 0.0, 1.0) *
        clampValue((2.0 - std::abs(lateral_error)) / 2.0, 0.0, 1.0);
      const double along_floor =
        clampValue(
          target_along_speed + 0.90 * along_floor_blend,
          0.0,
          carrier_tracking_speed_limit_);
      desired_along_velocity = std::max(desired_along_velocity, along_floor);
      const double lateral_budget = std::sqrt(std::max(
        0.0,
        carrier_tracking_speed_limit_ * carrier_tracking_speed_limit_ -
        along_floor * along_floor));
      desired_lateral_velocity =
        clampValue(desired_lateral_velocity, -lateral_budget, lateral_budget);
    }
    // Prefer keeping the lateral command unscaled by the final norm clamp; cap along-speed
    // just enough to fit within the tracking speed limit when lateral is saturated.
    if (passive_target_mode_ && (tracking_lateral_priority > 0.5 || tracking_hard_lateral || tracking_lat_strong)) {
      const double max_along_for_lateral = std::sqrt(std::max(
        0.0,
        carrier_tracking_speed_limit_ * carrier_tracking_speed_limit_ -
        desired_lateral_velocity * desired_lateral_velocity));
      desired_along_velocity = std::min(desired_along_velocity, max_along_for_lateral);
    }
    Eigen::Vector3d corridor_aligned_velocity =
      track_direction * desired_along_velocity +
      lateral_direction * desired_lateral_velocity;
    corridor_aligned_velocity.z() = desired_velocity.z();
    double smoothing_gain = 0.04;
    if (passive_target_mode_ && tracking_z_in_band && distance < 10.5) {
      smoothing_gain = 0.02;
    }
    const Eigen::Vector3d smooth_velocity =
      corridor_aligned_velocity - smoothing_gain * (carrier_velocity - target_velocity);
    carrier_velocity_command_ =
      clampNorm(smooth_velocity, carrier_tracking_speed_limit_);
    mini_velocity_command_.setZero();
    // When cross-track is still large, bias the position setpoint toward "directly under mini"
    // to aggressively collapse terminal lateral error before attempting corridor-style docking.
    const double pull_in_blend =
      tracking_lat_strong
      ? clampValue((std::abs(lateral_error) - 0.20) / 0.60, 0.0, 1.0)
      : clampValue((std::abs(lateral_error) - 0.35) / 0.85, 0.0, 1.0);
    carrier_position_setpoint_ =
      (1.0 - pull_in_blend) * intercept_position +
      pull_in_blend * direct_position;
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
  if (passive_target_mode_ && rendezvous_corridor_initialized_) {
    rendezvous_corridor_axis_ =
      computeLosDirection(horizontalOnly(target_velocity), rendezvous_corridor_axis_);
    rendezvous_corridor_anchor_ = poseToEigen(mini_pose_);
  }
  const Eigen::Vector3d fallback_track_direction =
    computeLosDirection(horizontalOnly(target_velocity), -horizontalOnly(terminal_error));
  const Eigen::Vector3d fallback_lateral_direction(
    -fallback_track_direction.y(), fallback_track_direction.x(), 0.0);
  const double fallback_terminal_lateral_error =
    horizontalOnly(terminal_error).dot(fallback_lateral_direction);
  Eigen::Vector3d track_direction =
    computeLosDirection(horizontalOnly(target_velocity), -horizontalOnly(terminal_error));
  Eigen::Vector3d lateral_direction(-track_direction.y(), track_direction.x(), 0.0);
  const double terminal_z_band_min = 0.25;
  const double terminal_z_band_max = 0.95;
  const double terminal_distance = terminal_error.norm();
  const double terminal_along_error =
    horizontalOnly(terminal_error).dot(track_direction);
  double terminal_lateral_error =
    horizontalOnly(terminal_error).dot(lateral_direction);

  const bool corridor_docking_active =
    passive_target_mode_ &&
    rendezvous_corridor_initialized_ &&
    std::abs(terminal_lateral_error) < 1.20;
  bool corridor_docking_control_active = corridor_docking_active;

  if (passive_target_mode_ && rendezvous_corridor_initialized_ && !corridor_docking_active) {
    track_direction =
      computeLosDirection(horizontalOnly(target_velocity), -horizontalOnly(terminal_error));
    lateral_direction = Eigen::Vector3d(-track_direction.y(), track_direction.x(), 0.0);
    terminal_lateral_error = horizontalOnly(terminal_error).dot(lateral_direction);
  }
	  const double far_blend = clampValue((terminal_distance - 0.35) / 2.0, 0.0, 1.0);
	  // When still far from the terminal point, allow DOCKING to use a higher speed limit
	  // (up to tracking) so the carrier does not fall behind the mini and drift out of the
	  // <=10m evaluation window before accumulating hold time.
	  const double effective_docking_speed_limit =
	    passive_target_mode_
	    ? clampValue(
	      carrier_docking_speed_limit_ +
	      far_blend * (carrier_tracking_speed_limit_ - carrier_docking_speed_limit_),
	      carrier_docking_speed_limit_,
	      carrier_tracking_speed_limit_)
	    : carrier_docking_speed_limit_;
	  const double lateral_hold_blend =
	    clampValue((std::abs(terminal_lateral_error) - 0.18) / 0.55, 0.0, 1.0);
  const double terminal_rel_z_target =
    passive_target_mode_ ? std::max(terminal_relative_position_.z(), 0.25) : terminal_relative_position_.z();
  const double staged_target_z =
    terminal_rel_z_target +
    (0.72 - terminal_rel_z_target) * std::max(far_blend, lateral_hold_blend);
  const double staged_along_target = 0.95 * far_blend;
  Eigen::Vector3d staged_target = terminal_relative_position_;
  staged_target.z() = terminal_rel_z_target;
  staged_target.x() += track_direction.x() * staged_along_target;
  staged_target.y() += track_direction.y() * staged_along_target;
  staged_target.z() = staged_target_z;
  const Eigen::Vector3d pos_error = relative_position - staged_target;
  const double terminal_blend = clampValue((terminal_distance - 0.22) / 1.4, 0.0, 1.0);
    const double terminal_along_error_now =
      horizontalOnly(terminal_error).dot(track_direction);
    const double terminal_lateral_error_now =
      horizontalOnly(terminal_error).dot(lateral_direction);
  const double terminal_along_speed_now =
    horizontalOnly(vel_error).dot(track_direction);
  const double terminal_lateral_speed_now =
    horizontalOnly(vel_error).dot(lateral_direction);
  const double terminal_relative_speed_now = vel_error.norm();
  const double target_along_speed_now = horizontalOnly(target_velocity).dot(track_direction);
  const double target_lateral_speed_now = horizontalOnly(target_velocity).dot(lateral_direction);
  const Eigen::Vector3d desired_vel =
    Eigen::Vector3d(
      0.10 * target_velocity.x(),
      0.05 * target_velocity.y(),
      0.00 * target_velocity.z()) * terminal_blend;

  const Eigen::Vector3d control_input = backstepping_controller_->computeControl(
    pos_error, vel_error, desired_vel, 1.0 / control_rate_);

  if (passive_target_mode_) {
    if (corridor_docking_active) {
      const double capture_release_core =
        0.42 * clampValue((0.65 - std::abs(terminal_along_error_now)) / 0.65, 0.0, 1.0) +
        0.26 * clampValue((0.30 - std::abs(terminal_error.z())) / 0.30, 0.0, 1.0) +
        0.18 * clampValue((0.70 - std::abs(vel_error.z())) / 0.70, 0.0, 1.0) +
        0.14 * clampValue((2.20 - terminal_relative_speed_now) / 2.20, 0.0, 1.0);
      const double capture_release_score =
        clampValue((1.40 - std::abs(terminal_along_error_now)) / 1.40, 0.0, 1.0) *
        capture_release_core;
      const double sync_release_core =
        0.50 * clampValue((0.65 - std::abs(terminal_lateral_error_now)) / 0.65, 0.0, 1.0) +
        0.25 * clampValue((0.35 - std::abs(terminal_error.z())) / 0.35, 0.0, 1.0) +
        0.13 * clampValue((1.00 - std::abs(vel_error.z())) / 1.00, 0.0, 1.0) +
        0.12 * clampValue((0.70 - terminal_relative_speed_now) / 0.70, 0.0, 1.0);
      const double sync_release_score =
        clampValue((4.00 - std::abs(terminal_along_error_now)) / 4.00, 0.0, 1.0) *
        sync_release_core;
      const double corridor_release_lateral_gate =
        clampValue((1.80 - std::abs(terminal_lateral_error_now)) / 1.60, 0.0, 1.0);
      const double corridor_release_score =
        corridor_release_lateral_gate * std::max(capture_release_score, sync_release_score);
      corridor_release_score_filtered_ =
        0.55 * corridor_release_score_filtered_ + 0.45 * corridor_release_score;
      if (corridor_release_score_filtered_ > 0.44) {
        corridor_release_armed_ = true;
      } else if (corridor_release_score_filtered_ < 0.34) {
        corridor_release_armed_ = false;
      }

      const bool corridor_release_geometry_ok =
        std::abs(fallback_terminal_lateral_error) < 1.35;
      const bool corridor_release_candidate =
        corridor_release_armed_ &&
        corridor_release_geometry_ok &&
        terminal_distance < 6.6 &&
        std::abs(terminal_lateral_error_now) < 1.10 &&
        std::abs(terminal_error.z()) < 0.65;
      if (corridor_release_candidate && corridor_release_score_filtered_ > 0.48) {
        corridor_release_accept_counter_++;
      } else if (!corridor_release_candidate || corridor_release_score_filtered_ < 0.40) {
        corridor_release_accept_counter_ = 0;
      } else if (corridor_release_accept_counter_ > 0) {
        corridor_release_accept_counter_--;
      }

      const bool corridor_release_to_terminal =
        corridor_release_geometry_ok && (
        (
          terminal_distance < 2.0 &&
          terminal_along_error_now > 1.2 &&
          terminal_along_error_now < 2.1 &&
          std::abs(terminal_lateral_error_now) < 0.20 &&
          std::abs(terminal_error.z()) < 0.10 &&
          std::abs(terminal_along_speed_now) < 0.45 &&
          std::abs(terminal_lateral_speed_now) < 0.20 &&
          std::abs(vel_error.z()) < 0.05 &&
          terminal_relative_speed_now < 0.60) ||
        (corridor_release_candidate && corridor_release_accept_counter_ >= 3) ||
        (corridor_release_score_filtered_ > 0.66 &&
        std::abs(fallback_terminal_lateral_error) < 0.85));
      const bool corridor_release_for_hold_metrics =
        corridor_release_geometry_ok &&
        // Release corridor earlier so the non-corridor terminal controller has enough time
        // to finish shrinking the XY error before the experiment ends.
        terminal_distance < 4.8 &&
        relative_position.z() > terminal_z_band_min &&
        relative_position.z() < terminal_z_band_max &&
        std::abs(terminal_lateral_error_now) < 0.25 &&
        std::abs(terminal_error.z()) < 0.55 &&
        terminal_relative_speed_now < 2.80 &&
        std::abs(terminal_lateral_speed_now) < 0.75;
      if (corridor_release_to_terminal || corridor_release_for_hold_metrics) {
        rendezvous_corridor_initialized_ = false;
        corridor_release_score_filtered_ = 0.0;
        corridor_release_armed_ = false;
        corridor_release_accept_counter_ = 0;
        corridor_docking_control_active = false;
      }
    } else {
      corridor_release_score_filtered_ = 0.0;
      corridor_release_armed_ = false;
      corridor_release_accept_counter_ = 0;
    }

    if (corridor_docking_control_active) {
      const Eigen::Vector3d carrier_position = poseToEigen(carrier_pose_);
      const Eigen::Vector3d mini_position = poseToEigen(mini_pose_);
      const Eigen::Vector3d corridor_track_direction = rendezvous_corridor_axis_;
      const Eigen::Vector3d corridor_lateral_direction(
        -corridor_track_direction.y(), corridor_track_direction.x(), 0.0);
      const double mini_progress =
        horizontalOnly(mini_position - rendezvous_corridor_anchor_).dot(corridor_track_direction);
      const double carrier_progress =
        horizontalOnly(carrier_position - rendezvous_corridor_anchor_).dot(corridor_track_direction);
      const double progress_gap = std::max(0.0, mini_progress - carrier_progress);
      const double carrier_lateral_error =
        horizontalOnly(carrier_position - rendezvous_corridor_anchor_).dot(corridor_lateral_direction);
      const double carrier_along_speed = horizontalOnly(carrier_velocity).dot(corridor_track_direction);
      const double carrier_lateral_speed = horizontalOnly(carrier_velocity).dot(corridor_lateral_direction);
      const double target_along_speed = horizontalOnly(target_velocity).dot(corridor_track_direction);
      const double target_lateral_speed = horizontalOnly(target_velocity).dot(corridor_lateral_direction);
      const double corridor_final_pull_blend =
        clampValue((2.2 - terminal_distance) / 1.2, 0.0, 1.0) *
        clampValue((0.25 - std::abs(terminal_lateral_error_now)) / 0.25, 0.0, 1.0) *
        clampValue((0.10 - std::abs(terminal_error.z())) / 0.10, 0.0, 1.0) *
        clampValue((0.60 - terminal_relative_speed_now) / 0.60, 0.0, 1.0) *
        clampValue((terminal_along_error_now - 0.9) / 1.2, 0.0, 1.0);
      const double docking_speed_limit =
        clampValue(
          carrier_docking_speed_limit_ +
          far_blend * (carrier_approach_speed_limit_ - carrier_docking_speed_limit_),
          carrier_docking_speed_limit_,
          carrier_approach_speed_limit_);
      const double corridor_lead =
        clampValue(0.35 + 0.55 * far_blend, 0.20, 0.90);
      const double corridor_catchup =
        clampValue(0.60 * progress_gap, 0.0, 18.0);
      Eigen::Vector3d corridor_target =
        rendezvous_corridor_anchor_ +
        corridor_track_direction * (carrier_progress + corridor_lead + corridor_catchup) +
        terminal_relative_position_;
      corridor_target.z() = mini_position.z() - terminal_rel_z_target;
      const Eigen::Vector3d corridor_error = corridor_target - carrier_position;
      const double corridor_along_error = horizontalOnly(corridor_error).dot(corridor_track_direction);
      const double corridor_vertical_error = corridor_error.z();

      Eigen::Vector3d desired_velocity =
        target_velocity +
        corridor_track_direction * clampValue(
          0.42 * corridor_along_error -
          0.26 * (carrier_along_speed - target_along_speed) +
          corridor_final_pull_blend *
          clampValue(0.36 * terminal_along_error_now, 0.0, 0.55),
          -1.0,
          1.5) +
        corridor_lateral_direction * clampValue(
          -0.32 * carrier_lateral_error -
          0.24 * (carrier_lateral_speed - target_lateral_speed),
          -(0.18 - 0.10 * corridor_final_pull_blend),
          +(0.18 - 0.10 * corridor_final_pull_blend)) +
        Eigen::Vector3d(
          0.0,
          0.0,
          clampValue(
            0.44 * corridor_vertical_error -
            0.18 * (carrier_velocity.z() - target_velocity.z()),
            -0.35,
            0.35));
      // Near the metric evaluation window (<=10m, z in band), the corridor-mode lateral term
      // (based on the corridor anchor frame) can allow the *terminal-frame* lateral relative speed
      // to stay biased, causing the terminal lateral error to overshoot past 0.2m and reset the
      // hold timer. Add a small terminal-frame PD to damp that overshoot.
      if (
        passive_target_mode_ &&
        terminal_distance < 10.0 &&
        relative_position.z() > terminal_z_band_min &&
        relative_position.z() < terminal_z_band_max &&
        std::abs(terminal_lateral_error_now) < 0.35)
      {
        const double z_margin_blend =
          clampValue((relative_position.z() - terminal_z_band_min) / 0.12, 0.0, 1.0) *
          clampValue((terminal_z_band_max - relative_position.z()) / 0.12, 0.0, 1.0);
        const double metric_blend =
          clampValue((10.0 - terminal_distance) / 4.0, 0.0, 1.0) *
          clampValue((0.35 - std::abs(terminal_lateral_error_now)) / 0.35, 0.0, 1.0) *
          z_margin_blend;
        const double lateral_limit = (0.18 - 0.10 * corridor_final_pull_blend);
        const double terminal_lateral_p = 0.65;
        const double terminal_lateral_d = 1.10;
        const double terminal_lateral_delta =
          clampValue(
            terminal_lateral_p * terminal_lateral_error_now +
            terminal_lateral_d * terminal_lateral_speed_now,
            -0.40,
            0.40);
        const double desired_lateral_cmd =
          clampValue(target_lateral_speed + terminal_lateral_delta, -lateral_limit, lateral_limit);
        desired_velocity +=
          metric_blend *
          lateral_direction * (desired_lateral_cmd - desired_velocity.dot(lateral_direction));
      }
      const double desired_along_velocity =
        clampValue(desired_velocity.dot(track_direction), 0.0, docking_speed_limit);
      const double desired_lateral_velocity =
        clampValue(
        desired_velocity.dot(lateral_direction),
        -(0.12 - 0.07 * corridor_final_pull_blend),
        +(0.12 - 0.07 * corridor_final_pull_blend));
      Eigen::Vector3d corridor_aligned_velocity =
        track_direction * desired_along_velocity +
        lateral_direction * desired_lateral_velocity;
      corridor_aligned_velocity.z() = desired_velocity.z();
      carrier_velocity_command_ =
        clampNorm(corridor_aligned_velocity, docking_speed_limit);
      mini_velocity_command_.setZero();
      carrier_position_setpoint_ = corridor_target;
      mini_position_setpoint_ = poseToEigen(mini_pose_);

      carrier_cmd.linear.x = carrier_velocity_command_(0);
      carrier_cmd.linear.y = carrier_velocity_command_(1);
      carrier_cmd.linear.z = carrier_velocity_command_(2);
      carrier_cmd.angular.z = 0.0;

      mini_cmd.linear.x = mini_velocity_command_(0);
      mini_cmd.linear.y = mini_velocity_command_(1);
      mini_cmd.linear.z = mini_velocity_command_(2);
      mini_cmd.angular.z = 0.0;
      return;
    }

    // Docking: command carrier in the target track frame, with explicit relative-speed targets.
    const Eigen::Vector3d horizontal_error = horizontalOnly(pos_error);
    const Eigen::Vector3d horizontal_rel_velocity = horizontalOnly(vel_error);
    const double along_error = horizontal_error.dot(track_direction);
    const double lateral_error = horizontal_error.dot(lateral_direction);
    const double along_rel_speed = horizontal_rel_velocity.dot(track_direction);
    const double lateral_rel_speed = horizontal_rel_velocity.dot(lateral_direction);
    const double close_blend = clampValue((2.2 - terminal_distance) / 2.2, 0.0, 1.0);
    const double tight_blend = clampValue((0.9 - terminal_distance) / 0.9, 0.0, 1.0);
    const double sync_entry_blend = clampValue((1.6 - terminal_distance) / 1.6, 0.0, 1.0);
    const double sync_lateral_blend = clampValue((0.45 - std::abs(lateral_error)) / 0.45, 0.0, 1.0);
    const double sync_vertical_blend = clampValue((0.24 - std::abs(pos_error.z())) / 0.24, 0.0, 1.0);
    const double sync_along_blend = clampValue((1.2 - std::abs(along_error)) / 1.2, 0.0, 1.0);
    const double sync_blend =
      passive_target_mode_ ?
      sync_entry_blend * sync_lateral_blend * sync_vertical_blend * sync_along_blend :
      0.0;
    const double safety_band_z_blend =
      clampValue((relative_position.z() - 0.25) / 0.25, 0.0, 1.0) *
      clampValue((0.95 - relative_position.z()) / 0.25, 0.0, 1.0);
    const double safety_band_hold_blend =
      passive_target_mode_ ?
      clampValue((10.0 - terminal_distance) / 4.0, 0.0, 1.0) *
      clampValue((0.55 - std::abs(terminal_lateral_error_now)) / 0.55, 0.0, 1.0) *
      safety_band_z_blend :
      0.0;
    const double safety_band_along_hold_blend =
      passive_target_mode_ ?
      clampValue((0.35 - std::abs(terminal_lateral_error)) / 0.35, 0.0, 1.0) *
      safety_band_z_blend :
      0.0;
    const double band_lock_blend =
      passive_target_mode_ ?
      clampValue((9.0 - terminal_distance) / 2.0, 0.0, 1.0) *
      clampValue((0.20 - std::abs(terminal_lateral_error)) / 0.20, 0.0, 1.0) *
      safety_band_z_blend :
      0.0;
    const double hold_lock_blend =
      terminal_lat_hold_latch_counter_ > 0 ? 1.0 : band_lock_blend;
    const bool docking_lat_hold_active =
      passive_target_mode_ && terminal_lat_hold_latch_counter_ > 0;
    const double terminal_along_capture_blend =
      passive_target_mode_ ?
      sync_blend *
      clampValue((0.25 - std::abs(terminal_lateral_error)) / 0.25, 0.0, 1.0) *
      clampValue((0.30 - std::abs(terminal_error.z())) / 0.30, 0.0, 1.0) *
      clampValue((terminal_along_error - 0.5) / 2.5, 0.0, 1.0) :
      0.0;

    const double base_desired_rel_along_limit =
      clampValue(1.35 - 0.78 * close_blend, 0.18, 1.35);
    const double base_desired_rel_lateral_limit =
      clampValue(1.00 - 0.64 * close_blend, 0.16, 1.00);
    const double base_desired_rel_vertical_limit =
      clampValue(0.42 - 0.22 * close_blend, 0.10, 0.42);
    const double desired_rel_along_limit_raw =
      (1.0 - sync_blend) * base_desired_rel_along_limit +
      sync_blend * (0.12 + 0.10 * terminal_along_capture_blend);
    const double desired_rel_along_limit =
      (1.0 - hold_lock_blend) *
      ((1.0 - safety_band_along_hold_blend) * desired_rel_along_limit_raw +
      safety_band_along_hold_blend * 0.12) +
      hold_lock_blend * 0.05;
    const double desired_rel_lateral_limit =
      (1.0 - safety_band_hold_blend) *
      ((1.0 - sync_blend) * base_desired_rel_lateral_limit + sync_blend * 0.04) +
      safety_band_hold_blend * 0.03;
    const double desired_rel_vertical_limit =
      (1.0 - sync_blend) * base_desired_rel_vertical_limit + sync_blend * 0.05;

    const double desired_rel_along =
      clampValue(
      0.55 * far_blend * (1.0 - 0.78 * sync_blend) -
      (0.80 + 0.36 * sync_blend) * along_error,
      -desired_rel_along_limit,
      desired_rel_along_limit);
    const double desired_rel_lateral =
      clampValue(
      -(1.22 + 0.54 * sync_blend) * lateral_error,
      -desired_rel_lateral_limit,
      desired_rel_lateral_limit);
    const double desired_rel_vertical =
      clampValue(
      -(0.98 + 0.42 * sync_blend) * pos_error.z(),
      -desired_rel_vertical_limit,
      desired_rel_vertical_limit);

    const double along_speed_error = along_rel_speed - desired_rel_along;
    const double lateral_speed_error = lateral_rel_speed - desired_rel_lateral;
    const double vertical_speed_error = vel_error.z() - desired_rel_vertical;

    const double base_along_correction_limit =
      clampValue(2.1 - 0.9 * close_blend - 0.3 * tight_blend, 0.60, 2.1);
    const double base_lateral_correction_limit =
      clampValue(1.4 - 0.7 * close_blend - 0.2 * tight_blend, 0.35, 1.4);
    const double base_vertical_correction_limit =
      clampValue(0.72 - 0.34 * close_blend, 0.18, 0.72);
    const double along_correction_limit_raw =
      (1.0 - sync_blend) * base_along_correction_limit +
      sync_blend * (0.42 + 0.22 * terminal_along_capture_blend);
    const double along_correction_limit =
      (1.0 - hold_lock_blend) *
      ((1.0 - safety_band_along_hold_blend) * along_correction_limit_raw +
      safety_band_along_hold_blend * 0.35) +
      hold_lock_blend * 0.18;
    const double lateral_correction_limit =
      (1.0 - safety_band_hold_blend) *
      ((1.0 - sync_blend) * base_lateral_correction_limit + sync_blend * 0.18) +
      safety_band_hold_blend * 0.12;
    const double vertical_correction_limit =
      (1.0 - sync_blend) * base_vertical_correction_limit + sync_blend * 0.12;

    const double along_correction =
      clampValue(
      (1.04 - 0.08 * sync_blend + 0.34 * terminal_along_capture_blend) * along_speed_error +
      (0.20 + 0.32 * terminal_along_capture_blend) * along_error,
      -along_correction_limit,
      along_correction_limit);
    const double lateral_correction =
      clampValue(
      (1.20 - 0.24 * sync_blend) * lateral_speed_error + 0.26 * lateral_error,
      -lateral_correction_limit,
      lateral_correction_limit);
    const double vertical_correction =
      clampValue(
      (0.96 - 0.18 * sync_blend) * vertical_speed_error + 0.20 * pos_error.z(),
      -vertical_correction_limit,
      vertical_correction_limit);

    Eigen::Vector3d commanded_velocity =
      target_velocity +
      track_direction * along_correction +
      lateral_direction * lateral_correction +
      Eigen::Vector3d(0.0, 0.0, vertical_correction);
	    commanded_velocity += control_input * (0.04 - 0.03 * close_blend);
	    commanded_velocity -= (0.34 + 0.30 * sync_blend) * (carrier_velocity - target_velocity);
	    const double commanded_along_velocity =
	      clampValue(commanded_velocity.dot(track_direction), 0.0, effective_docking_speed_limit);
	    double hold_along_velocity = commanded_along_velocity;
	    if (docking_lat_hold_active) {
      // In hold mode we want low *relative* speeds, but still track the mini feedforward
      // velocity so the terminal lateral error doesn't drift when the mini is moving.
      const double hold_along_delta_limit = 0.30;
      hold_along_velocity =
        clampValue(
          hold_along_velocity - target_along_speed_now,
          -hold_along_delta_limit,
          hold_along_delta_limit) + target_along_speed_now;
	      hold_along_velocity =
	        clampValue(hold_along_velocity, 0.0, effective_docking_speed_limit);
	    }
    const double non_corridor_lateral_limit =
      clampValue(0.55 + 0.30 * std::abs(lateral_error), 0.55, 1.60);
    const double safety_hold_lateral_limit =
      clampValue(
      (1.0 - safety_band_hold_blend) * non_corridor_lateral_limit +
      safety_band_hold_blend * 0.18,
      0.16,
      non_corridor_lateral_limit);
	    const double lateral_command_limit =
	      corridor_docking_active ?
	      0.18 :
	      (1.0 - hold_lock_blend) * safety_hold_lateral_limit + hold_lock_blend * 0.12;
    const double commanded_lateral_velocity =
      clampValue(
      commanded_velocity.dot(lateral_direction),
      -lateral_command_limit,
      lateral_command_limit);
	    const double safety_hold_lateral_target_limit =
	      (1.0 - hold_lock_blend) * 0.24 + hold_lock_blend * 0.12;
    const double safety_hold_lateral_gain =
      (1.0 - hold_lock_blend) * 0.85 + hold_lock_blend * 1.15;
    const double safety_hold_lateral_damping =
      (1.0 - hold_lock_blend) * 0.95 + hold_lock_blend * 1.40;
	    const double safety_hold_lateral_target =
	      clampValue(
	      -safety_hold_lateral_gain * lateral_error -
	      safety_hold_lateral_damping * lateral_rel_speed,
	      -safety_hold_lateral_target_limit,
	      safety_hold_lateral_target_limit);
	    double blended_lateral_velocity =
	      (1.0 - safety_band_hold_blend) * commanded_lateral_velocity +
	      safety_band_hold_blend * safety_hold_lateral_target;
	    if (passive_target_mode_ && !corridor_docking_active) {
	      // Use an explicit terminal-frame PD for lateral control in DOCKING (not only during
	      // the <0.2m latch). This makes large cross-track convergence less dependent on the
	      // backstepping controller's staged target coupling and tends to reduce run-to-run
	      // variance in whether we ever reach the <=0.2m window.
	      const double terminal_lateral_abs = std::abs(terminal_lateral_error_now);
	      const double aggressive_blend =
	        clampValue((terminal_lateral_abs - 0.30) / 0.90, 0.0, 1.0);
	      const double lateral_p =
	        (1.0 - aggressive_blend) * 0.70 + aggressive_blend * 0.95;
	      const double lateral_d =
	        (1.0 - aggressive_blend) * 0.95 + aggressive_blend * 0.72;
	      const double lateral_delta_limit =
	        clampValue(
	          0.30 +
	          0.55 * terminal_lateral_abs +
	          0.20 * std::abs(terminal_lateral_speed_now),
	          0.30,
	          2.20);
	      const double lateral_delta =
	        clampValue(
	          lateral_p * terminal_lateral_error_now +
	          lateral_d * terminal_lateral_speed_now,
	          -lateral_delta_limit,
	          lateral_delta_limit);
	      blended_lateral_velocity =
	        clampValue(
	          target_lateral_speed_now + lateral_delta,
	          -non_corridor_lateral_limit,
	          non_corridor_lateral_limit);
	    }
	    if (docking_lat_hold_active) {
	      // In terminal lat-hold we want to regulate the *metric* cross-track directly:
	      // controller_terminal_lateral_error uses terminal_error, not staged pos_error.
	      // Use a damped PD on terminal_lateral_error_now / terminal_lateral_speed_now to
	      // prevent overshoot that breaks the >=0.30s hold window.
	      const double hold_lateral_delta_limit =
	        clampValue(0.35 + 0.35 * std::abs(terminal_lateral_speed_now), 0.35, 1.80);
	      const double hold_lateral_p = 0.55;
	      const double hold_lateral_d = 0.95;
	      const double hold_lateral_delta =
	        clampValue(
	          hold_lateral_p * terminal_lateral_error_now +
	          hold_lateral_d * terminal_lateral_speed_now,
	          -hold_lateral_delta_limit,
	          hold_lateral_delta_limit);
	      blended_lateral_velocity = target_lateral_speed_now + hold_lateral_delta;
	    }
    Eigen::Vector3d corridor_aligned_velocity =
      track_direction * hold_along_velocity +
      lateral_direction * blended_lateral_velocity;
    corridor_aligned_velocity.z() = commanded_velocity.z();

    const double base_correction_speed_limit =
      clampValue(2.4 - 0.9 * close_blend - 0.3 * tight_blend, 0.7, 2.4);
    const double correction_speed_limit =
      (1.0 - sync_blend) * base_correction_speed_limit + sync_blend * 0.55;
	    Eigen::Vector3d correction_velocity = corridor_aligned_velocity - target_velocity;
	    correction_velocity = clampNorm(correction_velocity, correction_speed_limit);
	    carrier_velocity_command_ =
	      clampNorm(target_velocity + correction_velocity, effective_docking_speed_limit);
	    // Keep the relative height inside the evaluation z-band when we are very close, otherwise
	    // we can briefly hit low lateral error but immediately lose the band due to vertical overshoot.
	    if (passive_target_mode_) {
	      const double z_guard_min = 0.28;
	      const double z_guard_max = 0.92;
	      if (relative_position.z() < z_guard_min) {
	        carrier_velocity_command_.z() = std::min(carrier_velocity_command_.z(), 0.0);
	      } else if (relative_position.z() > z_guard_max) {
	        carrier_velocity_command_.z() = std::max(carrier_velocity_command_.z(), 0.0);
	      }
	    }
	    mini_velocity_command_.setZero();
	    const double preview_horizon =
	      (0.24 + 0.16 * (1.0 - close_blend)) * (1.0 - 0.45 * sync_blend);
	    const Eigen::Vector3d preview_offset =
      track_direction * desired_rel_along * preview_horizon +
      lateral_direction *
      (corridor_docking_active ? 0.35 * desired_rel_lateral : desired_rel_lateral) *
      preview_horizon +
      Eigen::Vector3d(0.0, 0.0, desired_rel_vertical * preview_horizon);
    if (corridor_docking_active) {
      const Eigen::Vector3d mini_position = poseToEigen(mini_pose_);
      const double corridor_progress =
        horizontalOnly(mini_position - rendezvous_corridor_anchor_).dot(track_direction);
      const Eigen::Vector3d corridor_point =
        rendezvous_corridor_anchor_ +
        track_direction * (corridor_progress + 0.45 * preview_horizon * carrier_docking_speed_limit_) +
        Eigen::Vector3d(0.0, 0.0, mini_position.z());
      carrier_position_setpoint_ = corridor_point - staged_target - preview_offset;
    } else {
      carrier_position_setpoint_ =
        poseToEigen(mini_pose_) + target_velocity * preview_horizon * far_blend -
        staged_target - preview_offset;
    }
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

std::array<double, 20> DockingController::getControllerDebug() const
{
  return controller_debug_;
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
