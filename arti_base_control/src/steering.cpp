#include <arti_base_control/steering.h>
#include <arti_base_control/utils.h>
#include <arti_base_control/wheel.h>
#include <cmath>
#include <functional>

namespace arti_base_control
{
IdealAckermannSteering::IdealAckermannSteering(const ros::NodeHandle& nh)
  : config_server_(nh)
{
  config_server_.setCallback(std::bind(&IdealAckermannSteering::reconfigure, this, std::placeholders::_1));
}

double IdealAckermannSteering::computeWheelSteeringAngle(const Wheel& wheel, const double steering_position) const
{
  return wheel.computeIdealWheelSteeringAngle(steering_position, config_.icr_x);
}

double IdealAckermannSteering::computeWheelSteeringVelocity(
  const Wheel& wheel, double steering_position, double steering_velocity) const
{
  // In case the wheel is on the ICR line (which makes no sense for a steered wheel), default to no steering:
  if (wheel.position_x_ == config_.icr_x)
  {
    return 0.0;
  }

  // This is the time derivative of the steering angle formula:
  const double x = wheel.position_x_ - config_.icr_x;
  const double x2 = x * x;
  const double sin_steering_angle = std::sin(steering_position);
  const double k = x * std::cos(steering_position) - wheel.hinge_position_y_ * sin_steering_angle;

  return steering_velocity * x2 / (k * k + x2 * sin_steering_angle * sin_steering_angle);
}

double IdealAckermannSteering::computeSteeringPosition(const Wheel& wheel, double wheel_steering_angle) const
{
  // In case the wheel is on the ICR line (which makes no sense for a steered wheel), default to no steering:
  if (wheel.position_x_ == config_.icr_x)
  {
    return 0.0;
  }

  // This is the inverse of the steering angle formula:
  const double sin_wheel_steering_angle = std::sin(wheel_steering_angle);
  const double x = wheel.position_x_ - config_.icr_x;
  return normalizeSteeringAngle(
    std::atan2(x * sin_wheel_steering_angle,
               x * std::cos(wheel_steering_angle) + wheel.hinge_position_y_ * sin_wheel_steering_angle));
}

void IdealAckermannSteering::reconfigure(IdealAckermannSteeringConfig& config)
{
  config_ = config;
}


FourBarLinkageSteering::FourBarLinkageSteering(const ros::NodeHandle& nh)
  : config_server_(nh)
{
  config_server_.setCallback(std::bind(&FourBarLinkageSteering::reconfigure, this, std::placeholders::_1));
}

double FourBarLinkageSteering::computeWheelSteeringAngle(const Wheel& wheel, double steering_position) const
{
  // FIXME
}

double FourBarLinkageSteering::computeWheelSteeringVelocity(
  const Wheel& wheel, double steering_position, double steering_velocity) const
{
  // FIXME
}

double FourBarLinkageSteering::computeSteeringPosition(const Wheel& wheel, double wheel_steering_angle) const
{
  // FIXME
}

void FourBarLinkageSteering::reconfigure(FourBarLinkageSteeringConfig& config)
{
  config_ = config;
}

double FourBarLinkageSteering::computeFloatingLinkLength(const Wheel& wheel) const
{
  const double x_c = config_.steering_shaft_x + config_.steering_crank_length * std::cos(config_.steering_crank_angle);
  const double y_c = config_.steering_shaft_y + config_.steering_crank_length * std::sin(config_.steering_crank_angle);
  const double x_a = wheel.position_x_ + config_.wheel_steering_arm_length * std::cos(config_.wheel_steering_arm_angle);
  const double y_a
    = wheel.hinge_position_y_ + config_.wheel_steering_arm_length * std::sin(config_.wheel_steering_arm_angle);

  return std::hypot(x_a - x_c, y_a - y_c);
}
}
