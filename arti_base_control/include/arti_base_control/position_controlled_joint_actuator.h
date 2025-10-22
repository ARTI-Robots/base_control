#ifndef ARTI_BASE_CONTROL_POSITION_CONTROLLED_JOINT_ACTUATOR_H
#define ARTI_BASE_CONTROL_POSITION_CONTROLLED_JOINT_ACTUATOR_H

#include <arti_base_control/joint_sensor.h>
#include <arti_base_control/types.h>
#include <boost/optional.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <std_msgs/msg/float64.hpp>


namespace arti_base_control
{
class PositionControlledJointActuator : public JointSensor
{
public:
  virtual ~PositionControlledJointActuator() = default;
  virtual void setPosition(double position) = 0;
};

class PublishingPositionControlledJointActuator : public PositionControlledJointActuator
{
public:
  PublishingPositionControlledJointActuator(
    const rclcpp::Node::SharedPtr& node_handle,
    const PositionControlledJointActuatorPtr& joint_actuator);

  JointState getState(const rclcpp::Time& time) override;

  boost::optional<double> getSupplyVoltage() override;

  void setPosition(double position) override;

protected:
  PublishingJointSensor publishing_joint_sensor_;
  PositionControlledJointActuatorPtr joint_actuator_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr position_command_publisher_;
};
}

#endif //ARTI_BASE_CONTROL_POSITION_CONTROLLED_JOINT_ACTUATOR_H
