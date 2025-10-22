#include <arti_base_control/joint_sensor.h>
#include <arti_base_control/utils.h>
#include <std_msgs/msg/float64.hpp>

namespace arti_base_control
{
PublishingJointSensor::PublishingJointSensor(
  const rclcpp::Node::SharedPtr& node_handle,
  const JointSensorPtr& joint_sensor)
  : joint_sensor_(joint_sensor)
{
  position_publisher_ = node_handle->create_publisher<std_msgs::msg::Float64>("position", 10);
  velocity_publisher_ = node_handle->create_publisher<std_msgs::msg::Float64>("velocity", 10);
}

JointState PublishingJointSensor::getState(const rclcpp::Time& time)
{
  const JointState state = joint_sensor_->getState(time);

  std_msgs::msg::Float64 pos_msg;
  pos_msg.data = state.position;
  position_publisher_->publish(pos_msg);

  std_msgs::msg::Float64 vel_msg;
  vel_msg.data = state.position;
  velocity_publisher_->publish(vel_msg);

  return state;
}

boost::optional<double> PublishingJointSensor::getSupplyVoltage()
{
  return joint_sensor_->getSupplyVoltage();
}
}
