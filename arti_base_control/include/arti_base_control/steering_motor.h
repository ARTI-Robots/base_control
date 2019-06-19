#ifndef ARTI_BASE_CONTROL_STEERING_MOTOR_H
#define ARTI_BASE_CONTROL_STEERING_MOTOR_H

#include <arti_base_control/types.h>
#include <boost/optional.hpp>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <vesc_motor/types.h>
#include <vesc_motor/vesc_steering_motor.h>

namespace arti_base_control
{
class SteeringMotor
{
public:
  virtual ~SteeringMotor() = default;

  virtual double getPosition(const ros::Time& time) = 0;
  virtual double getVelocity(const ros::Time& time) = 0;

  virtual void setPosition(double position) = 0;

  virtual boost::optional<double> getSupplyVoltage() = 0;
};

class VescSteeringMotor : public SteeringMotor
{
public:
  VescSteeringMotor(ros::NodeHandle& private_nh, const vesc_motor::DriverFactoryPtr& driver_factory,
                    double control_interval);

  double getPosition(const ros::Time& time) override;
  double getVelocity(const ros::Time& time) override;

  void setPosition(double position) override;

  boost::optional<double> getSupplyVoltage() override;

protected:
  vesc_motor::VescSteeringMotor motor_;
};

class PublishingSteeringMotor : public SteeringMotor
{
public:
  PublishingSteeringMotor(ros::NodeHandle& private_nh, const SteeringMotorPtr& motor);

  double getPosition(const ros::Time& time) override;
  double getVelocity(const ros::Time& time) override;

  void setPosition(double position) override;

  boost::optional<double> getSupplyVoltage() override;

protected:
  SteeringMotorPtr motor_;
  ros::Publisher position_sent_publisher_;
  ros::Publisher position_received_publisher_;
  ros::Publisher velocity_received_publisher_;
};
}

#endif //ARTI_BASE_CONTROL_STEERING_MOTOR_H
