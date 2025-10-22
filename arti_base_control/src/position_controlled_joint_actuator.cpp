#include <arti_base_control/position_controlled_joint_actuator.h>
#include <arti_base_control/utils.h>
#include <std_msgs/msg/float64.hpp>

namespace arti_base_control
{
PublishingPositionControlledJointActuator::PublishingPositionControlledJointActuator(
  const rclcpp::Node::SharedPtr& node_handle,
  const PositionControlledJointActuatorPtr& joint_actuator)
  : PositionControlledJointActuator(*joint_actuator),
    publishing_joint_sensor_(node_handle, joint_actuator), 
    joint_actuator_(joint_actuator) 
{
  position_command_publisher_ =
    node_handle->create_publisher<std_msgs::msg::Float64>("position_command", 1);
}

JointState PublishingPositionControlledJointActuator::getState(const rclcpp::Time& time)
{
  return publishing_joint_sensor_.getState(time);
}

boost::optional<double> PublishingPositionControlledJointActuator::getSupplyVoltage()
{
  return publishing_joint_sensor_.getSupplyVoltage();
}

void PublishingPositionControlledJointActuator::setPosition(double position)
{
  joint_actuator_->setPosition(position);

  std_msgs::msg::Float64 msg;
  msg.data = position;
  position_command_publisher_->publish(msg);
}
}
