#include <arti_base_control/position_controlled_joint_actuator.h>
#include <arti_base_control/utils.h>
#include <std_msgs/Float64.h>

namespace arti_base_control
{
PublishingPositionControlledJointActuator::PublishingPositionControlledJointActuator(
  ros::NodeHandle& node_handle, const PositionControlledJointActuatorPtr& joint_actuator)
  : publishing_joint_sensor_(node_handle, joint_actuator), joint_actuator_(joint_actuator),
    position_command_publisher_(node_handle.advertise<std_msgs::Float64>("position_command", 1))
{
}

JointState PublishingPositionControlledJointActuator::getState(const ros::Time& time)
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
  position_command_publisher_.publish(makeDataMsg<std_msgs::Float64>(position));
}
}
