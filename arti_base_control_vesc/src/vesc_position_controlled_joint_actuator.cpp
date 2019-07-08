#include <arti_base_control_vesc/vesc_position_controlled_joint_actuator.h>

namespace arti_base_control_vesc
{
VescPositionControlledJointActuator::VescPositionControlledJointActuator(
  ros::NodeHandle& private_nh, const vesc_motor::DriverFactoryPtr& driver_factory, double control_interval)
  : motor_(private_nh, driver_factory, std::chrono::duration<double>(control_interval))
{
}

arti_base_control::JointState VescPositionControlledJointActuator::getState(const ros::Time& time)
{
  return arti_base_control::JointState(motor_.getPosition(time), motor_.getVelocity(time));
}

void VescPositionControlledJointActuator::setPosition(double position)
{
  motor_.setPosition(position);
}

boost::optional<double> VescPositionControlledJointActuator::getSupplyVoltage()
{
  return motor_.getSupplyVoltage();
}
}
