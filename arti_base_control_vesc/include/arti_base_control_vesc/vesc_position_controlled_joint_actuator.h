#ifndef ARTI_BASE_CONTROL_VESC_VESC_POSITION_CONTROLLED_JOINT_ACTUATOR_H
#define ARTI_BASE_CONTROL_VESC_VESC_POSITION_CONTROLLED_JOINT_ACTUATOR_H

#include <arti_base_control/position_controlled_joint_actuator.h>
#include <dynamic_reconfigure/server.h>
#include <mutex>
#include <opencv2/video/tracking.hpp>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <arti_base_control_vesc/SteeringMotorConfig.h>
#include <arti_base_control_vesc/vesc_motor.h>

namespace arti_base_control_vesc
{
class VescPositionControlledJointActuator : public arti_base_control::PositionControlledJointActuator, public VescMotor
{
public:
  VescPositionControlledJointActuator(ros::NodeHandle& private_nh, const vesc_driver::DriverFactoryPtr& driver_factory,
                    double control_interval);

  arti_base_control::JointState getState(const ros::Time& time) override;

  void setPosition(double position) override;

  boost::optional<double> getSupplyVoltage() override;

protected:
  void reconfigure(SteeringMotorConfig& config);
  void processMotorControllerState(const vesc_driver::MotorControllerState& state) override;

  std::mutex config_mutex_;
  dynamic_reconfigure::Server<SteeringMotorConfig> reconfigure_server_;
  SteeringMotorConfig config_;
};
}

#endif //ARTI_BASE_CONTROL_VESC_VESC_POSITION_CONTROLLED_JOINT_ACTUATOR_H
