#include <arti_base_control_vesc/vesc_joint_actuator_factory.h>
#include <arti_base_control_vesc/vesc_position_controlled_joint_actuator.h>
#include <arti_base_control_vesc/vesc_velocity_controlled_joint_actuator.h>
#include <vesc_motor/driver_factory.h>
#include <vesc_motor/transport_factory.h>

namespace arti_base_control_vesc
{
VescJointActuatorFactory::VescJointActuatorFactory(const ros::NodeHandle& nh, double control_interval, bool use_mockup)
  : control_interval_(control_interval)
{
  driver_factory_ = std::make_shared<vesc_motor::DriverFactory>(std::make_shared<vesc_motor::TransportFactory>(nh),
                                                                use_mockup);
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
