#ifndef ARTI_BASE_CONTROL_VELOCITY_CONTROLLED_JOINT_ACTUATOR_H
#define ARTI_BASE_CONTROL_VELOCITY_CONTROLLED_JOINT_ACTUATOR_H

#include <arti_base_control/joint_sensor.h>
#include <arti_base_control/types.h>
#include <boost/optional.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <std_msgs/msg/float64.hpp>

namespace arti_base_control
{
class VelocityControlledJointActuator : public JointSensor
{
public:
  virtual ~VelocityControlledJointActuator() = default;
  virtual void setVelocity(double velocity) = 0;
  virtual void brake(double current) = 0;
};

class PublishingVelocityControlledJointActuator : public VelocityControlledJointActuator
{
public:
  PublishingVelocityControlledJointActuator(
    rclcpp::Node::SharedPtr& private_nh,
    const VelocityControlledJointActuatorPtr& joint_actuator);

  JointState getState(const rclcpp::Time& time) override;

  void setVelocity(double velocity) override;

  void brake(double current) override;

  boost::optional<double> getSupplyVoltage() override;

private:
  PublishingJointSensor publishing_joint_sensor_;
  VelocityControlledJointActuatorPtr joint_actuator_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr velocity_command_publisher_;
};
}

#endif //ARTI_BASE_CONTROL_VELOCITY_CONTROLLED_JOINT_ACTUATOR_H
