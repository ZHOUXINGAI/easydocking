#include <chrono>
#include <cctype>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <easydocking_msgs/msg/docking_command.hpp>
#include <easydocking_msgs/msg/docking_status.hpp>
#include <easydocking_msgs/msg/relative_pose.hpp>

#include <easydocking_control/docking_controller.hpp>

namespace easydocking
{

class DockingControllerNode : public rclcpp::Node
{
public:
  explicit DockingControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("docking_controller_node", options),
    controller_(std::make_unique<DockingController>()),
    has_carrier_odom_(false),
    has_mini_odom_(false)
  {
    declareParameters();
    configureController();
    createInterfaces();

    const double control_rate = this->get_parameter("control_rate").as_double();
    const auto period =
      std::chrono::milliseconds(static_cast<int>(1000.0 / std::max(control_rate, 1.0)));
    control_timer_ = this->create_wall_timer(
      period, std::bind(&DockingControllerNode::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "Docking controller node ready");
  }

private:
  void declareParameters()
  {
    this->declare_parameter("approach_distance", 5.0);
    this->declare_parameter("tracking_distance", 2.0);
    this->declare_parameter("docking_distance", 0.5);
    this->declare_parameter("control_rate", 50.0);
    this->declare_parameter("k1", 1.0);
    this->declare_parameter("k2", 2.0);
    this->declare_parameter("adaptive_gain", 0.5);
    this->declare_parameter("guidance_gain", 1.0);
    this->declare_parameter("idle_hover_altitude", 1.5);
    this->declare_parameter("world_frame", "map");
    this->declare_parameter("passive_target_mode", false);
    this->declare_parameter("desired_relative_position", std::vector<double>{0.0, 0.0, 0.6});
    this->declare_parameter("terminal_relative_position", std::vector<double>{0.0, 0.0, 0.2});
    this->declare_parameter("carrier_approach_speed_limit", 3.0);
    this->declare_parameter("carrier_tracking_speed_limit", 2.2);
    this->declare_parameter("carrier_docking_speed_limit", 1.0);
    this->declare_parameter("intercept_lookahead", 1.5);
    this->declare_parameter("docking_speed_threshold", 1.0);
  }

  void configureController()
  {
    const auto desired_relative_position =
      this->get_parameter("desired_relative_position").as_double_array();
    const auto terminal_relative_position =
      this->get_parameter("terminal_relative_position").as_double_array();
    controller_->setParameters(
      this->get_parameter("approach_distance").as_double(),
      this->get_parameter("tracking_distance").as_double(),
      this->get_parameter("docking_distance").as_double(),
      this->get_parameter("k1").as_double(),
      this->get_parameter("k2").as_double(),
      this->get_parameter("adaptive_gain").as_double(),
      this->get_parameter("guidance_gain").as_double());
    controller_->setIdleHoverAltitude(this->get_parameter("idle_hover_altitude").as_double());
    controller_->setPassiveTargetMode(this->get_parameter("passive_target_mode").as_bool());
    if (desired_relative_position.size() == 3) {
      controller_->setDesiredRelativePosition(Eigen::Vector3d(
        desired_relative_position[0], desired_relative_position[1], desired_relative_position[2]));
    }
    if (terminal_relative_position.size() == 3) {
      controller_->setTerminalRelativePosition(Eigen::Vector3d(
        terminal_relative_position[0], terminal_relative_position[1], terminal_relative_position[2]));
    }
    controller_->setCarrierLimits(
      this->get_parameter("carrier_approach_speed_limit").as_double(),
      this->get_parameter("carrier_tracking_speed_limit").as_double(),
      this->get_parameter("carrier_docking_speed_limit").as_double());
    controller_->setInterceptLookahead(this->get_parameter("intercept_lookahead").as_double());
    controller_->setDockingSpeedThreshold(this->get_parameter("docking_speed_threshold").as_double());
  }

  void createInterfaces()
  {
    carrier_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/carrier/odom", 10,
      std::bind(&DockingControllerNode::carrierOdomCallback, this, std::placeholders::_1));
    mini_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/mini/odom", 10,
      std::bind(&DockingControllerNode::miniOdomCallback, this, std::placeholders::_1));
    command_sub_ = this->create_subscription<easydocking_msgs::msg::DockingCommand>(
      "/docking/command", 10,
      std::bind(&DockingControllerNode::commandCallback, this, std::placeholders::_1));
    auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    command_latched_sub_ = this->create_subscription<easydocking_msgs::msg::DockingCommand>(
      "/docking/command_latched", latched_qos,
      std::bind(&DockingControllerNode::latchedCommandCallback, this, std::placeholders::_1));
    command_latched_pub_ =
      this->create_publisher<easydocking_msgs::msg::DockingCommand>("/docking/command_latched", latched_qos);

    carrier_velocity_pub_ =
      this->create_publisher<geometry_msgs::msg::TwistStamped>("/carrier/setpoint/velocity", 10);
    mini_velocity_pub_ =
      this->create_publisher<geometry_msgs::msg::TwistStamped>("/mini/setpoint/velocity", 10);
    carrier_pose_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>("/carrier/setpoint/pose", 10);
    mini_pose_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>("/mini/setpoint/pose", 10);
    relative_pose_pub_ =
      this->create_publisher<easydocking_msgs::msg::RelativePose>("/docking/relative_pose", 10);
    status_pub_ =
      this->create_publisher<easydocking_msgs::msg::DockingStatus>("/docking/status", 10);
  }

  void carrierOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    carrier_odom_ = *msg;
    has_carrier_odom_ = true;
  }

  void miniOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    mini_odom_ = *msg;
    has_mini_odom_ = true;
  }

  void commandCallback(const easydocking_msgs::msg::DockingCommand::SharedPtr msg)
  {
    handleCommand(*msg);
    command_latched_pub_->publish(*msg);
  }

  void latchedCommandCallback(const easydocking_msgs::msg::DockingCommand::SharedPtr msg)
  {
    handleCommand(*msg);
  }

  void handleCommand(const easydocking_msgs::msg::DockingCommand & msg)
  {
    std::string command = msg.command;
    for (auto & ch : command) {
      ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
    }

    if (command == "START") {
      if (!controller_->isDockingActive()) {
        controller_->startDocking();
        RCLCPP_INFO(this->get_logger(), "Received START, docking activated");
      }
    } else if (command == "STOP") {
      if (controller_->isDockingActive()) {
        controller_->stopDocking();
        RCLCPP_INFO(this->get_logger(), "Received STOP, docking deactivated");
      }
    } else if (command == "RESET") {
      controller_->reset();
      RCLCPP_INFO(this->get_logger(), "Received RESET, controller reset");
    }
  }

  void controlLoop()
  {
    if (!has_carrier_odom_ || !has_mini_odom_) {
      return;
    }

    controller_->updatePoses(
      carrier_odom_.pose.pose, carrier_odom_.twist.twist,
      mini_odom_.pose.pose, mini_odom_.twist.twist);

    geometry_msgs::msg::Twist carrier_cmd;
    geometry_msgs::msg::Twist mini_cmd;
    controller_->computeControlCommands(carrier_cmd, mini_cmd);

    publishVelocitySetpoint("carrier", carrier_cmd, carrier_velocity_pub_);
    publishVelocitySetpoint("mini", mini_cmd, mini_velocity_pub_);
    publishPoseSetpoint(
      controller_->getCarrierPositionSetpoint(), carrier_odom_.pose.pose.orientation,
      carrier_pose_pub_);
    publishPoseSetpoint(
      controller_->getMiniPositionSetpoint(), mini_odom_.pose.pose.orientation,
      mini_pose_pub_);
    publishRelativePose();
    publishStatus();
  }

  void publishVelocitySetpoint(
    const std::string & frame_id,
    const geometry_msgs::msg::Twist & twist,
    const rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr & publisher)
  {
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = frame_id;
    msg.twist = twist;
    publisher->publish(msg);
  }

  void publishPoseSetpoint(
    const Eigen::Vector3d & position,
    const geometry_msgs::msg::Quaternion & orientation,
    const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr & publisher)
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = this->get_parameter("world_frame").as_string();
    msg.pose.position.x = position.x();
    msg.pose.position.y = position.y();
    msg.pose.position.z = position.z();
    msg.pose.orientation = orientation;
    publisher->publish(msg);
  }

  void publishRelativePose()
  {
    easydocking_msgs::msg::RelativePose msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = this->get_parameter("world_frame").as_string();

    const Eigen::Vector3d relative_position = controller_->getRelativePosition();
    const Eigen::Vector3d relative_velocity = controller_->getRelativeVelocity();

    msg.position.x = relative_position.x();
    msg.position.y = relative_position.y();
    msg.position.z = relative_position.z();
    msg.linear_velocity.x = relative_velocity.x();
    msg.linear_velocity.y = relative_velocity.y();
    msg.linear_velocity.z = relative_velocity.z();
    msg.orientation = mini_odom_.pose.pose.orientation;
    msg.distance = controller_->getRelativeDistance();

    relative_pose_pub_->publish(msg);
  }

  void publishStatus()
  {
    easydocking_msgs::msg::DockingStatus status;
    status.header.stamp = this->get_clock()->now();
    status.header.frame_id = this->get_parameter("world_frame").as_string();
    status.phase = controller_->getCurrentPhaseName();
    status.is_active = controller_->isDockingActive();
    status.relative_distance = controller_->getRelativeDistance();

    const Eigen::Vector3d relative_position = controller_->getRelativePosition();
    const Eigen::Vector3d relative_velocity = controller_->getRelativeVelocity();
    const Eigen::Vector3d disturbance_estimate = controller_->getDisturbanceEstimate();
    const Eigen::Vector3d target_velocity_estimate = controller_->getEstimatedTargetVelocity();

    status.relative_position.x = relative_position.x();
    status.relative_position.y = relative_position.y();
    status.relative_position.z = relative_position.z();
    status.relative_velocity.x = relative_velocity.x();
    status.relative_velocity.y = relative_velocity.y();
    status.relative_velocity.z = relative_velocity.z();
    status.disturbance_estimate.x = disturbance_estimate.x();
    status.disturbance_estimate.y = disturbance_estimate.y();
    status.disturbance_estimate.z = disturbance_estimate.z();
    status.estimated_target_velocity.x = target_velocity_estimate.x();
    status.estimated_target_velocity.y = target_velocity_estimate.y();
    status.estimated_target_velocity.z = target_velocity_estimate.z();

    status_pub_->publish(status);
  }

  std::unique_ptr<DockingController> controller_;
  nav_msgs::msg::Odometry carrier_odom_;
  nav_msgs::msg::Odometry mini_odom_;
  bool has_carrier_odom_;
  bool has_mini_odom_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr carrier_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr mini_odom_sub_;
  rclcpp::Subscription<easydocking_msgs::msg::DockingCommand>::SharedPtr command_sub_;
  rclcpp::Subscription<easydocking_msgs::msg::DockingCommand>::SharedPtr command_latched_sub_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr carrier_velocity_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr mini_velocity_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr carrier_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mini_pose_pub_;
  rclcpp::Publisher<easydocking_msgs::msg::RelativePose>::SharedPtr relative_pose_pub_;
  rclcpp::Publisher<easydocking_msgs::msg::DockingStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<easydocking_msgs::msg::DockingCommand>::SharedPtr command_latched_pub_;

  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace easydocking

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<easydocking::DockingControllerNode>());
  rclcpp::shutdown();
  return 0;
}
