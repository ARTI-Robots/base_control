#include <arti_base_control_vesc/vesc_velocity_controlled_joint_actuator.h>

namespace arti_base_control_vesc
{
VescVelocityControlledJointActuator::VescVelocityControlledJointActuator(
  ros::NodeHandle& private_nh, const vesc_driver::DriverFactoryPtr& driver_factory, double control_interval)
  : motor_(private_nh, driver_factory, std::chrono::duration<double>(control_interval))
{
}

arti_base_control::JointState VescVelocityControlledJointActuator::getState(const ros::Time& time)
{
  const double velocity = motor_.getVelocity(time);

  if (time > last_velocity_time_)
  {
    if (!last_velocity_time_.isZero())
    {
      const double time_difference = (time - last_velocity_time_).toSec();
      last_position_ += (last_velocity_ + velocity) * 0.5 * time_difference;  // Assumes constant acceleration
    }

    last_velocity_ = velocity;
    last_velocity_time_ = time;
  }

  return arti_base_control::JointState(last_position_, velocity);
}

void VescVelocityControlledJointActuator::setVelocity(const double velocity)
{
  motor_.setVelocity(velocity);
}

void VescVelocityControlledJointActuator::brake(const double current)
{
  motor_.brake(current);
}

boost::optional<double> VescVelocityControlledJointActuator::getSupplyVoltage()
{
  return motor_.getSupplyVoltage();
}
}
