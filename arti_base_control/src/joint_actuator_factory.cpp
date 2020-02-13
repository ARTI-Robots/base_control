#include <arti_base_control/joint_actuator_factory.h>
#include <arti_base_control/velocity_controlled_joint_actuator.h>
#include <arti_base_control/position_controlled_joint_actuator.h>

namespace arti_base_control
{
PublishingJointActuatorFactory::PublishingJointActuatorFactory(const JointActuatorFactoryPtr& factory)
  : factory_(factory)
{
}

void PublishingJointActuatorFactory::init(
  const ros::NodeHandle& /*private_nh*/, double /*control_interval*/, bool /*use_mockup*/)
{
  // as this is only a wrapper there is nothing to do for the initalisation
}

PositionControlledJointActuatorPtr PublishingJointActuatorFactory::createPositionControlledJointActuator(
  ros::NodeHandle& private_nh)
{
  return std::make_shared<PublishingPositionControlledJointActuator>(
    private_nh, factory_->createPositionControlledJointActuator(private_nh));
}

VelocityControlledJointActuatorPtr PublishingJointActuatorFactory::createVelocityControlledJointActuator(
  ros::NodeHandle& private_nh)
{
  return std::make_shared<PublishingVelocityControlledJointActuator>(
    private_nh, factory_->createVelocityControlledJointActuator(private_nh));
}
}
