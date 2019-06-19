#include <arti_base_control/drive_motor.h>
#include <arti_base_control/utils.h>
#include <std_msgs/Float64.h>

namespace arti_base_control
{
VescDriveMotor::VescDriveMotor(ros::NodeHandle& private_nh, const vesc_motor::DriverFactoryPtr& driver_factory,
                               double control_interval)
  : motor_(private_nh, driver_factory, std::chrono::duration<double>(control_interval))
{
}

double VescDriveMotor::getPosition(const ros::Time& time)
{
  if (time > last_velocity_time_)
  {
    getVelocity(time);
  }

  return last_position_;
}

double VescDriveMotor::getVelocity(const ros::Time& time)
{
  const double velocity = motor_.getVelocity(time);

  if (time > last_velocity_time_)
  {
    if (!last_velocity_time_.isZero())
    {
      const double time_difference = (time - last_velocity_time_).toSec();
      last_position_ += (last_velocity_ + velocity) * 0.5 * time_difference;  // Assumes constant acceleration
    }

    last_velocity_ = velocity;
    last_velocity_time_ = time;
  }

  return velocity;
}

void VescDriveMotor::setVelocity(const double velocity)
{
  motor_.setVelocity(velocity);
}

void VescDriveMotor::brake(const double current)
{
  motor_.brake(current);
}

boost::optional<double> VescDriveMotor::getSupplyVoltage()
{
  return motor_.getSupplyVoltage();
}

PublishingDriveMotor::PublishingDriveMotor(ros::NodeHandle& private_nh, const DriveMotorPtr& motor)
  : motor_(motor),
    velocity_sent_publisher_(private_nh.advertise<std_msgs::Float64>("velocity_sent", 1)),
    velocity_received_publisher_(private_nh.advertise<std_msgs::Float64>("velocity_received", 1)),
    position_received_publisher_(private_nh.advertise<std_msgs::Float64>("position_received", 1))
{
}

double PublishingDriveMotor::getPosition(const ros::Time& time)
{
  const double position = motor_->getPosition(time);
  position_received_publisher_.publish(makeDataMsg<std_msgs::Float64>(position));
  return position;
}

double PublishingDriveMotor::getVelocity(const ros::Time& time)
{
  const double velocity = motor_->getVelocity(time);
  velocity_received_publisher_.publish(makeDataMsg<std_msgs::Float64>(velocity));
  return velocity;
}

void PublishingDriveMotor::setVelocity(const double velocity)
{
  motor_->setVelocity(velocity);
  velocity_sent_publisher_.publish(makeDataMsg<std_msgs::Float64>(velocity));
}

void PublishingDriveMotor::brake(const double current)
{
  motor_->brake(current);
  velocity_sent_publisher_.publish(makeDataMsg<std_msgs::Float64>(0.0));
}

boost::optional<double> PublishingDriveMotor::getSupplyVoltage()
{
  return motor_->getSupplyVoltage();
}
}
