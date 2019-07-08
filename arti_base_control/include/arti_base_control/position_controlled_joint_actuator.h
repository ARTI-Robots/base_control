#ifndef ARTI_BASE_CONTROL_POSITION_CONTROLLED_JOINT_ACTUATOR_H
#define ARTI_BASE_CONTROL_POSITION_CONTROLLED_JOINT_ACTUATOR_H

#include <arti_base_control/joint_sensor.h>
#include <arti_base_control/types.h>
#include <boost/optional.hpp>
#include <ros/node_handle.h>
#include <ros/time.h>

namespace arti_base_control
{
class PositionControlledJointActuator : public JointSensor
{
public:
  virtual void setPosition(double position) = 0;
};

class PublishingPositionControlledJointActuator : public PositionControlledJointActuator
{
public:
  PublishingPositionControlledJointActuator(
    ros::NodeHandle& node_handle, const PositionControlledJointActuatorPtr& joint_actuator);

  JointState getState(const ros::Time& time) override;

  boost::optional<double> getSupplyVoltage() override;

  void setPosition(double position) override;

protected:
  PublishingJointSensor publishing_joint_sensor_;
  PositionControlledJointActuatorPtr joint_actuator_;
  ros::Publisher position_command_publisher_;
};
}

#endif //ARTI_BASE_CONTROL_POSITION_CONTROLLED_JOINT_ACTUATOR_H
