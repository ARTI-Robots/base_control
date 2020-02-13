#ifndef ARTI_BASE_CONTROL_VESC_VESC_POSITION_CONTROLLED_JOINT_ACTUATOR_H
#define ARTI_BASE_CONTROL_VESC_VESC_POSITION_CONTROLLED_JOINT_ACTUATOR_H

#include <arti_base_control/position_controlled_joint_actuator.h>
#include <arti_base_control_vesc/vesc_steering_motor.h>

namespace arti_base_control_vesc
{
class VescPositionControlledJointActuator : public arti_base_control::PositionControlledJointActuator
{
public:
  VescPositionControlledJointActuator(ros::NodeHandle& private_nh, const vesc_driver::DriverFactoryPtr& driver_factory,
                    double control_interval);

  arti_base_control::JointState getState(const ros::Time& time) override;

  void setPosition(double position) override;

  boost::optional<double> getSupplyVoltage() override;

protected:
  arti_base_control_vesc::VescSteeringMotor motor_;
};
}

#endif //ARTI_BASE_CONTROL_VESC_VESC_POSITION_CONTROLLED_JOINT_ACTUATOR_H
