#ifndef ARTI_BASE_CONTROL_DRIVE_MOTOR_H
#define ARTI_BASE_CONTROL_DRIVE_MOTOR_H

#include <arti_base_control/types.h>
#include <boost/optional.hpp>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <vesc_motor/types.h>
#include <vesc_motor/vesc_drive_motor.h>

namespace arti_base_control
{
class DriveMotor
{
public:
  virtual ~DriveMotor() = default;

  virtual double getPosition(const ros::Time& time) = 0;
  virtual double getVelocity(const ros::Time& time) = 0;

  virtual void setVelocity(double velocity) = 0;

  virtual void brake(double current) = 0;

  virtual boost::optional<double> getSupplyVoltage() = 0;
};

class VescDriveMotor : public DriveMotor
{
public:
  VescDriveMotor(ros::NodeHandle& private_nh, const vesc_motor::DriverFactoryPtr& driver_factory,
                 double control_interval);

  double getPosition(const ros::Time& time) override;
  double getVelocity(const ros::Time& time) override;

  void setVelocity(double velocity) override;

  void brake(double current) override;

  boost::optional<double> getSupplyVoltage() override;

private:
  vesc_motor::VescDriveMotor motor_;
  double last_position_ = 0.0;
  double last_velocity_ = 0.0;
  ros::Time last_velocity_time_;
};

class PublishingDriveMotor : public DriveMotor
{
public:
  PublishingDriveMotor(ros::NodeHandle& private_nh, const DriveMotorPtr& motor);

  double getPosition(const ros::Time& time) override;
  double getVelocity(const ros::Time& time) override;

  void setVelocity(double velocity) override;

  void brake(double current) override;

  boost::optional<double> getSupplyVoltage() override;

private:
  DriveMotorPtr motor_;
  ros::Publisher velocity_sent_publisher_;
  ros::Publisher velocity_received_publisher_;
  ros::Publisher position_received_publisher_;
};
}

#endif //ARTI_BASE_CONTROL_DRIVE_MOTOR_H
