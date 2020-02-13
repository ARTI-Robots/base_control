#include <arti_base_control_vesc/vesc_joint_actuator_factory.h>
#include <arti_base_control_vesc/vesc_position_controlled_joint_actuator.h>
#include <arti_base_control_vesc/vesc_velocity_controlled_joint_actuator.h>
#include <vesc_driver/driver_factory.h>
#include <vesc_driver/transport_factory.h>
#include <pluginlib/class_list_macros.h>

namespace arti_base_control_vesc
{
VescJointActuatorFactory::VescJointActuatorFactory()
  : control_interval_(1.)
{
}

void VescJointActuatorFactory::init(const ros::NodeHandle& private_nh, double control_interval, bool use_mockup)
{
  control_interval_ = control_interval;

  driver_factory_ = std::make_shared<vesc_driver::DriverFactory>(
    std::make_shared<vesc_driver::TransportFactory>(private_nh), use_mockup);
}

arti_base_control::PositionControlledJointActuatorPtr VescJointActuatorFactory::createPositionControlledJointActuator(
  ros::NodeHandle& private_nh)
{
  const arti_base_control::PositionControlledJointActuatorPtr actuator
    = std::make_shared<VescPositionControlledJointActuator>(private_nh, driver_factory_, control_interval_);
  actuator->setPosition(0.0);
  return actuator;
}

arti_base_control::VelocityControlledJointActuatorPtr VescJointActuatorFactory::createVelocityControlledJointActuator(
  ros::NodeHandle& private_nh)
{
  const arti_base_control::VelocityControlledJointActuatorPtr actuator
    = std::make_shared<VescVelocityControlledJointActuator>(private_nh, driver_factory_, control_interval_);
  actuator->setVelocity(0.0);
  return actuator;
}
}

PLUGINLIB_EXPORT_CLASS(arti_base_control_vesc::VescJointActuatorFactory, arti_base_control::JointActuatorFactory)

