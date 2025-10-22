#ifndef ARTI_BASE_CONTROL_BASE_CONTROL_H
#define ARTI_BASE_CONTROL_BASE_CONTROL_H

#include <rclcpp/rclcpp.hpp>

#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32.hpp>
#include <arti_base_control_msgs/msg/odometry_calculation_info.hpp>

#include <arti_base_control/types.h>
#include <arti_base_control/vehicle.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pluginlib/class_loader.hpp>
#include <arti_base_control/joint_actuator_factory.h>

#include <optional>
#include <memory>
#include <string>

namespace arti_base_control
{

// Configuration structure replacing dynamic_reconfigure
struct BaseControlConfig
{
  // Output controls
  bool publish_odom{true};
  bool publish_tf{true};
  bool publish_executed_command{true};
  bool publish_motor_states{false};
  bool publish_joint_states{true};
  bool publish_supply_voltage{true};
  bool publish_calculation_info{false};

  // Execution
  bool execute_ackermann_commands{true};

  // Frames
  std::string odom_frame{"odom"};
  std::string base_frame{"base_link"};

  // Timing
  double odometry_rate{10.0};  // in Hz

  // Behavior
  bool use_mockup{false};

  // Covariances
  double odom_x_y_cov{0.0};
  double odom_yaw_cov{0.0};
  double odom_x_vel_cov{0.0};
  double odom_y_vel_cov{0.0};
  double odom_omega_cov{0.0};

  // Plugin configuration
  std::string motor_driver{"arti_base_control_vesc/VescJointActuatorFactory"};
};

class BaseControl
{
public:
  // In ROS 2 we use a shared Node pointer instead of NodeHandle
  explicit BaseControl(const rclcpp::Node::SharedPtr& private_nh);

protected:
  // Configuration update function (replacement for dynamic_reconfigure)
  void reconfigure(const BaseControlConfig& cfg);

  // Callbacks for velocity and Ackermann commands
  void processVelocityCommand(const geometry_msgs::msg::Twist::ConstSharedPtr& cmd_vel);
  void processAckermannCommand(const ackermann_msgs::msg::AckermannDrive::ConstSharedPtr& cmd_ackermann);

  // Periodic timer (no TimerEvent in ROS 2)
  void processOdomTimer();

  // Odometry helper functions
  void updateOdometry(
    const rclcpp::Time& time,
    const geometry_msgs::msg::Twist& velocity,
    arti_base_control_msgs::msg::OdometryCalculationInfo& odometry_calculation_info);

  void publishOdometry(const geometry_msgs::msg::Twist& velocity);
  void publishSupplyVoltage();

  // Node handle
  rclcpp::Node::SharedPtr private_nh_;

  // Current configuration
  BaseControlConfig config_;

  // Plugin loader
  pluginlib::ClassLoader<arti_base_control::JointActuatorFactory> plugin_loader_{
    "arti_base_control", "arti_base_control::JointActuatorFactory"};

  // Vehicle
  std::optional<Vehicle> vehicle_;

  // Odometry state
  rclcpp::Time odom_update_time_;
  geometry_msgs::msg::Pose2D odom_pose_;

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr executed_command_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr supply_voltage_pub_;
  rclcpp::Publisher<arti_base_control_msgs::msg::OdometryCalculationInfo>::SharedPtr calculation_infos_pub_;

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_twist_sub_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr cmd_ackermann_sub_;

  // Timer
  rclcpp::TimerBase::SharedPtr odom_timer_;


  // Parameter callback handle (to keep dynamic parameter updates alive)
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_cb_handle_;
};

}  // namespace arti_base_control

#endif  // ARTI_BASE_CONTROL_BASE_CONTROL_H
