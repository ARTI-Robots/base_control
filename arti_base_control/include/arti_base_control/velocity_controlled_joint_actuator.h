#ifndef ARTI_BASE_CONTROL_VELOCITY_CONTROLLED_JOINT_ACTUATOR_H
#define ARTI_BASE_CONTROL_VELOCITY_CONTROLLED_JOINT_ACTUATOR_H

#include <arti_base_control/joint_sensor.h>
#include <arti_base_control/types.h>
#include <boost/optional.hpp>
#include <ros/node_handle.h>
#include <ros/time.h>

namespace arti_base_control
{
class VelocityControlledJointActuator : public JointSensor
{
public:
  virtual void setVelocity(double velocity) = 0;

  virtual void brake(double current) = 0;
};

class PublishingVelocityControlledJointActuator : public VelocityControlledJointActuator
{
public:
  PublishingVelocityControlledJointActuator(
    ros::NodeHandle& private_nh, const VelocityControlledJointActuatorPtr& joint_actuator);

  JointState getState(const ros::Time& time) override;

  void setVelocity(double velocity) override;

  void brake(double current) override;

  boost::optional<double> getSupplyVoltage() override;

private:
  PublishingJointSensor publishing_joint_sensor_;
  VelocityControlledJointActuatorPtr joint_actuator_;
  ros::Publisher velocity_command_publisher_;
};
}

#endif //ARTI_BASE_CONTROL_VELOCITY_CONTROLLED_JOINT_ACTUATOR_H
