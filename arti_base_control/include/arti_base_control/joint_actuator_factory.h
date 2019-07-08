#ifndef ARTI_BASE_CONTROL_JOINT_ACTUATOR_FACTORY_H
#define ARTI_BASE_CONTROL_JOINT_ACTUATOR_FACTORY_H

#include <arti_base_control/types.h>
#include <ros/node_handle.h>

namespace arti_base_control
{
class JointActuatorFactory
{
public:
  virtual ~JointActuatorFactory() = default;

  virtual PositionControlledJointActuatorPtr createPositionControlledJointActuator(ros::NodeHandle& private_nh) = 0;
  virtual VelocityControlledJointActuatorPtr createVelocityControlledJointActuator(ros::NodeHandle& private_nh) = 0;
};

class PublishingJointActuatorFactory : public JointActuatorFactory
{
public:
  explicit PublishingJointActuatorFactory(const JointActuatorFactoryPtr& factory);

  PositionControlledJointActuatorPtr createPositionControlledJointActuator(ros::NodeHandle& private_nh) override;
  VelocityControlledJointActuatorPtr createVelocityControlledJointActuator(ros::NodeHandle& private_nh) override;

protected:
  JointActuatorFactoryPtr factory_;
};
}

#endif //ARTI_BASE_CONTROL_JOINT_ACTUATOR_FACTORY_H
