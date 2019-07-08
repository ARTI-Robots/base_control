#ifndef ARTI_BASE_CONTROL_VESC_VESC_POSITION_CONTROLLED_JOINT_ACTUATOR_H
#define ARTI_BASE_CONTROL_VESC_VESC_POSITION_CONTROLLED_JOINT_ACTUATOR_H

#include <arti_base_control/position_controlled_joint_actuator.h>
#include <vesc_motor/types.h>
#include <vesc_motor/vesc_steering_motor.h>

namespace arti_base_control_vesc
{
class VescPositionControlledJointActuator : public arti_base_control::PositionControlledJointActuator
{
public:
  VescPositionControlledJointActuator(ros::NodeHandle& private_nh, const vesc_motor::DriverFactoryPtr& driver_factory,
                    double control_interval);

  arti_base_control::JointState getState(const ros::Time& time) override;

  void setPosition(double position) override;

  boost::optional<double> getSupplyVoltage() override;

protected:
  vesc_motor::VescSteeringMotor motor_;
};
}

#endif //ARTI_BASE_CONTROL_VESC_VESC_POSITION_CONTROLLED_JOINT_ACTUATOR_H
