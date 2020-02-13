#ifndef ARTI_BASE_CONTROL_VESC_VESC_VELOCITY_CONTROLLED_JOINT_ACTUATOR_H
#define ARTI_BASE_CONTROL_VESC_VESC_VELOCITY_CONTROLLED_JOINT_ACTUATOR_H

#include <arti_base_control/velocity_controlled_joint_actuator.h>
#include <dynamic_reconfigure/server.h>
#include <mutex>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <arti_base_control_vesc/DriveMotorConfig.h>
#include <arti_base_control_vesc/vesc_motor.h>

namespace arti_base_control_vesc
{
class VescVelocityControlledJointActuator : public arti_base_control::VelocityControlledJointActuator, public VescMotor
{
public:
  VescVelocityControlledJointActuator(
    ros::NodeHandle& private_nh, const vesc_driver::DriverFactoryPtr& driver_factory, double control_interval);

  arti_base_control::JointState getState(const ros::Time& time) override;

  void setVelocity(double velocity) override;

  void brake(double current) override;

  boost::optional<double> getSupplyVoltage() override;

private:
  void processMotorControllerState(const vesc_driver::MotorControllerState& state) override;

  void reconfigure(DriveMotorConfig& config);
  double getVelocityConversionFactor() const;

  std::mutex config_mutex_;
  dynamic_reconfigure::Server<DriveMotorConfig> reconfigure_server_;
  DriveMotorConfig config_;

  double last_position_ = 0.0;
  double last_velocity_ = 0.0;
  ros::Time last_velocity_time_;
};
}

#endif //ARTI_BASE_CONTROL_VESC_VESC_VELOCITY_CONTROLLED_JOINT_ACTUATOR_H
