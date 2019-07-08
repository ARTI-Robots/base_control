#ifndef ARTI_BASE_CONTROL_VESC_VESC_JOINT_ACTUATOR_FACTORY_H
#define ARTI_BASE_CONTROL_VESC_VESC_JOINT_ACTUATOR_FACTORY_H

#include <arti_base_control/joint_actuator_factory.h>
#include <arti_base_control/types.h>
#include <vesc_motor/types.h>

namespace arti_base_control_vesc
{
class VescJointActuatorFactory : public arti_base_control::JointActuatorFactory
{
public:
  VescJointActuatorFactory(const ros::NodeHandle& nh, double control_interval, bool use_mockup);

  arti_base_control::PositionControlledJointActuatorPtr createPositionControlledJointActuator(
    ros::NodeHandle& private_nh) override;

  arti_base_control::VelocityControlledJointActuatorPtr createVelocityControlledJointActuator(
    ros::NodeHandle& private_nh) override;

protected:
  vesc_motor::DriverFactoryPtr driver_factory_;
  double control_interval_;
};

}

#endif //ARTI_BASE_CONTROL_VESC_VESC_JOINT_ACTUATOR_FACTORY_H