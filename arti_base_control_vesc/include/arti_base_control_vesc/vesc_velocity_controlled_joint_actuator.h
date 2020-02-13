#ifndef ARTI_BASE_CONTROL_VESC_VESC_VELOCITY_CONTROLLED_JOINT_ACTUATOR_H
#define ARTI_BASE_CONTROL_VESC_VESC_VELOCITY_CONTROLLED_JOINT_ACTUATOR_H

#include <arti_base_control/velocity_controlled_joint_actuator.h>
#include <arti_base_control_vesc/vesc_drive_motor.h>

namespace arti_base_control_vesc
{
class VescVelocityControlledJointActuator : public arti_base_control::VelocityControlledJointActuator
{
public:
  VescVelocityControlledJointActuator(
    ros::NodeHandle& private_nh, const vesc_driver::DriverFactoryPtr& driver_factory, double control_interval);

  arti_base_control::JointState getState(const ros::Time& time) override;

  void setVelocity(double velocity) override;

  void brake(double current) override;

  boost::optional<double> getSupplyVoltage() override;

private:
  arti_base_control_vesc::VescDriveMotor motor_;
  double last_position_ = 0.0;
  double last_velocity_ = 0.0;
  ros::Time last_velocity_time_;
};
}

#endif //ARTI_BASE_CONTROL_VESC_VESC_VELOCITY_CONTROLLED_JOINT_ACTUATOR_H
