#include <arti_base_control/velocity_controlled_joint_actuator.h>
#include <arti_base_control/utils.h>
#include <std_msgs/msg/float64.hpp>
#include <rclcpp/rclcpp.hpp>

namespace arti_base_control
{
PublishingVelocityControlledJointActuator::PublishingVelocityControlledJointActuator(
  rclcpp::Node::SharedPtr& private_nh,
  const VelocityControlledJointActuatorPtr& joint_actuator)
  : VelocityControlledJointActuator(*joint_actuator),
    publishing_joint_sensor_(private_nh, joint_actuator),  
    joint_actuator_(joint_actuator) 
{
  velocity_command_publisher_ =
    private_nh->create_publisher<std_msgs::msg::Float64>("velocity_command", 1);
}

JointState PublishingVelocityControlledJointActuator::getState(const rclcpp::Time& time)
{
  return publishing_joint_sensor_.getState(time);
}

boost::optional<double> PublishingVelocityControlledJointActuator::getSupplyVoltage()
{
  return publishing_joint_sensor_.getSupplyVoltage();
}

void PublishingVelocityControlledJointActuator::setVelocity(const double velocity)
{
  joint_actuator_->setVelocity(velocity);

  // Publish command to ROS2 topic
  std_msgs::msg::Float64 msg;
  msg.data = velocity;
  velocity_command_publisher_->publish(msg);
}

void PublishingVelocityControlledJointActuator::brake(const double current)
{
  joint_actuator_->brake(current);
  
  // Publish zero velocity command to indicate braking
  std_msgs::msg::Float64 msg;
  msg.data = 0.0;
  velocity_command_publisher_->publish(msg);
}
}
