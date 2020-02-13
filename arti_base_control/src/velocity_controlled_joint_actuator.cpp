#include <arti_base_control/velocity_controlled_joint_actuator.h>
#include <arti_base_control/utils.h>
#include <std_msgs/Float64.h>

namespace arti_base_control
{
PublishingVelocityControlledJointActuator::PublishingVelocityControlledJointActuator(
  ros::NodeHandle& private_nh, const VelocityControlledJointActuatorPtr& joint_actuator)
  : publishing_joint_sensor_(private_nh, joint_actuator), joint_actuator_(joint_actuator),
    velocity_command_publisher_(private_nh.advertise<std_msgs::Float64>("velocity_command", 1))
{
}

JointState PublishingVelocityControlledJointActuator::getState(const ros::Time& time)
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
  velocity_command_publisher_.publish(makeDataMsg<std_msgs::Float64>(velocity));
}

void PublishingVelocityControlledJointActuator::brake(const double current)
{
  joint_actuator_->brake(current);
  velocity_command_publisher_.publish(makeDataMsg<std_msgs::Float64>(0.0));
}
}
