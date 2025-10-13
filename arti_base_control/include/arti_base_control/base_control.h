#ifndef ARTI_BASE_CONTROL_BASE_CONTROL_H
#define ARTI_BASE_CONTROL_BASE_CONTROL_H

#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include <arti_base_control/BaseControlConfig.h>
#include <arti_base_control/types.h>
#include <arti_base_control_msgs/msg/odometry_calculation_info.hpp>
#include <arti_base_control/vehicle.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf/transform_broadcaster.h>
#include <pluginlib/class_loader.h>
#include <arti_base_control/joint_actuator_factory.h>

namespace arti_base_control
{
class BaseControl
{
public:
  explicit BaseControl(const rclcpp::Node& private_nh);

protected:
  void reconfigure(BaseControlConfig& config);

  void processVelocityCommand(const geometry_msgs::msg::Twist::ConstSharedPtr& cmd_vel);
  void processAckermannCommand(const ackermann_msgs::msg::AckermannDrive::ConstSharedPtr& cmd_ackermann);

  void processOdomTimerEvent(const rclcpp::TimerEvent& event);
  void updateOdometry(
    const rclcpp::Time& time, const geometry_msgs::msg::Twist& velocity,
    arti_base_control_msgs::msg::OdometryCalculationInfo& odometry_calculation_info);
  void publishOdometry(const geometry_msgs::msg::Twist& velocity);

  void publishSupplyVoltage();

  rclcpp::Node private_nh_;

  BaseControlConfig config_;
  dynamic_reconfigure::Server<BaseControlConfig> reconfigure_server_;

  pluginlib::ClassLoader<arti_base_control::JointActuatorFactory> plugin_loader_;

  boost::optional<Vehicle> vehicle_;

  rclcpp::Time odom_update_time_;

  geometry_msgs::msg::Pose2D odom_pose_;

  ros::Publisher odom_pub_;
  boost::optional<tf::TransformBroadcaster> tf_broadcaster_;
  ros::Publisher executed_command_pub_;
  ros::Publisher joint_states_pub_;
  ros::Publisher supply_voltage_pub_;
  ros::Publisher calculation_infos_pub_;

  ros::Subscriber cmd_vel_twist_sub_;
  ros::Subscriber cmd_ackermann_sub_;

  rclcpp::Timer odom_timer_;
};
}


#endif //ARTI_BASE_CONTROL_BASE_CONTROL_H
