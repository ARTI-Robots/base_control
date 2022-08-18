#include <arti_base_control/steering.h>
#include <angles/angles.h>
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

JointState IdealAckermannSteering::computeWheelSteeringState(const Wheel& wheel, const JointState& steering_state) const
{
  JointState wheel_steering_state;

  // In case the wheel is on the ICR line (which makes no sense for a steered wheel), default to no steering:
  if (wheel.position_x_ == config_.icr_x)
  {
    return wheel_steering_state;
  }

  wheel_steering_state.position = wheel.computeIdealWheelSteeringAngle(steering_state.position * config_.steering_scaling, config_.icr_x);

  // This is the time derivative of the steering angle formula:
  const double x = wheel.position_x_ - config_.icr_x;
  const double x2 = x * x;
  const double sin_steering_angle = std::sin(steering_state.position * config_.steering_scaling);
  const double k = x * std::cos(steering_state.position * config_.steering_scaling) - wheel.hinge_position_y_ * sin_steering_angle;
  wheel_steering_state.velocity = steering_state.velocity * config_.steering_scaling * x2 / (k * k + x2 * sin_steering_angle * sin_steering_angle);

  return wheel_steering_state;
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
               x * std::cos(wheel_steering_angle) + wheel.hinge_position_y_ * sin_wheel_steering_angle)) /
               config_.steering_scaling;
}

JointLimits IdealAckermannSteering::computeWheelSteeringLimits(const Wheel& /*wheel*/) const
{
  // Basically, there are no limits, but more than 90deg really makes no sense for Ackermann steering:
  return JointLimits(-M_PI_2, M_PI_2);
}

void IdealAckermannSteering::getJointStates(
  const JointState& steering_state, JointStates& joint_states) const
{
  if (!config_.steering_joint.empty())
  {
    joint_states[config_.steering_joint] = steering_state;
  }
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

JointState FourBarLinkageSteering::computeWheelSteeringState(const Wheel& wheel, const JointState& steering_state) const
{
  const double dt = 0.001;
  const double wheel_steering_angle_0
    = computeWheelSteeringAngle(wheel, steering_state.position);
  const double wheel_steering_angle_1
    = computeWheelSteeringAngle(wheel, steering_state.position + steering_state.velocity * dt);

  return JointState(wheel_steering_angle_0, (wheel_steering_angle_1 - wheel_steering_angle_0) / dt);
}

double FourBarLinkageSteering::computeSteeringPosition(const Wheel& wheel, double wheel_steering_angle) const
{
  // We do a little dirty trick here: when the wheel is on the right side, we assume that the geometry is mirrored:
  const double hinge_position_sign = (wheel.hinge_position_y_ < 0.0) ? -1.0 : 1.0;
  const double steering_crank_angle = config_.steering_crank_angle * hinge_position_sign;
  const double wheel_steering_arm_angle = config_.wheel_steering_arm_angle * hinge_position_sign + wheel_steering_angle;

  const double x_a = wheel.position_x_ + config_.wheel_steering_arm_length * std::cos(wheel_steering_arm_angle);
  const double y_a = wheel.hinge_position_y_ + config_.wheel_steering_arm_length * std::sin(wheel_steering_arm_angle);

  const double shaft_wheel_arm_x = x_a - config_.steering_shaft_x;
  const double shaft_wheel_arm_y = y_a - config_.steering_shaft_y;
  const double shaft_wheel_arm_length_squared
    = shaft_wheel_arm_x * shaft_wheel_arm_x + shaft_wheel_arm_y * shaft_wheel_arm_y;
  const double shaft_wheel_arm_length = std::sqrt(shaft_wheel_arm_length_squared);
  const double shaft_wheel_arm_angle = std::atan2(shaft_wheel_arm_y, shaft_wheel_arm_x);

  const double floating_link_length = computeFloatingLinkLength(wheel);

  const double shaft_wheel_arm_crank_angle_cos
    = (shaft_wheel_arm_length_squared + config_.steering_crank_length * config_.steering_crank_length
       - floating_link_length * floating_link_length)
      / (2.0 * shaft_wheel_arm_length * config_.steering_crank_length);

  // Use minmax here to make sure that acos returns a finite value; this assumes that the wheel_steering_angle is
  // within valid bounds:
  const double shaft_wheel_arm_crank_angle = std::acos(
    std::min(std::max(-1.0, shaft_wheel_arm_crank_angle_cos), 1.0));

  const double steering_position_pos
    = angles::normalize_angle(shaft_wheel_arm_angle + shaft_wheel_arm_crank_angle - steering_crank_angle);
  const double steering_position_neg
    = angles::normalize_angle(shaft_wheel_arm_angle - shaft_wheel_arm_crank_angle - steering_crank_angle);

  if (std::fabs(steering_position_pos) < std::fabs(steering_position_neg))
  {
    return steering_position_pos;
  }
  return steering_position_neg;
}

JointLimits FourBarLinkageSteering::computeWheelSteeringLimits(const Wheel& /*wheel*/) const
{
  /*
  // We do a little dirty trick here: when the wheel is on the right side, we assume that the geometry is mirrored:
  const double hinge_position_sign = (wheel.hinge_position_y_ < 0.0) ? -1.0 : 1.0;
  const double steering_crank_angle = config_.steering_crank_angle * hinge_position_sign;
  const double wheel_steering_arm_angle = config_.wheel_steering_arm_angle * hinge_position_sign;

  const double floating_link_length = computeFloatingLinkLength(wheel);

  const double min_shaft_wheel_arm_length = std::fabs(floating_link_length - config_.steering_crank_length);
  const double max_shaft_wheel_arm_length = floating_link_length + config_.steering_crank_length;

  const double min_hinge_crank_length = std::fabs(floating_link_length - config_.wheel_steering_arm_length);
  const double max_hinge_crank_length = floating_link_length + config_.wheel_steering_arm_length;

  // FIXME TODO: implement
   */

  return JointLimits(-M_PI_2, M_PI_2);

}

void FourBarLinkageSteering::getJointStates(
  const JointState& steering_state, JointStates& joint_states) const
{
  if (!config_.steering_shaft_joint.empty())
  {
    joint_states[config_.steering_shaft_joint] = steering_state;
  }
}

void FourBarLinkageSteering::reconfigure(FourBarLinkageSteeringConfig& config)
{
  config_ = config;
}

double FourBarLinkageSteering::computeWheelSteeringAngle(const Wheel& wheel, double steering_position) const
{
  // We do a little dirty trick here: when the wheel is on the right side, we assume that the geometry is mirrored:
  const double hinge_position_sign = (wheel.hinge_position_y_ < 0.0) ? -1.0 : 1.0;
  const double steering_crank_angle = config_.steering_crank_angle * hinge_position_sign + steering_position;
  const double wheel_steering_arm_angle = config_.wheel_steering_arm_angle * hinge_position_sign;

  const double x_c = config_.steering_shaft_x + config_.steering_crank_length * std::cos(steering_crank_angle);
  const double y_c = config_.steering_shaft_y + config_.steering_crank_length * std::sin(steering_crank_angle);

  const double hinge_crank_x = x_c - wheel.position_x_;
  const double hinge_crank_y = y_c - wheel.hinge_position_y_;
  const double hinge_crank_length_squared = hinge_crank_x * hinge_crank_x + hinge_crank_y * hinge_crank_y;
  const double hinge_crank_angle = std::atan2(hinge_crank_y, hinge_crank_x);

  const double floating_link_length = computeFloatingLinkLength(wheel);
  const double hinge_crank_arm_angle = std::acos(
    (hinge_crank_length_squared + config_.wheel_steering_arm_length * config_.wheel_steering_arm_length
     - floating_link_length * floating_link_length)
    / (2.0 * std::sqrt(hinge_crank_length_squared) * config_.wheel_steering_arm_length));

  const double wheel_steering_angle_pos
    = angles::normalize_angle(hinge_crank_angle + hinge_crank_arm_angle - wheel_steering_arm_angle);
  const double wheel_steering_angle_neg
    = angles::normalize_angle(hinge_crank_angle - hinge_crank_arm_angle - wheel_steering_arm_angle);

  if (std::fabs(wheel_steering_angle_pos) < std::fabs(wheel_steering_angle_neg))
  {
    return wheel_steering_angle_pos;
  }
  return wheel_steering_angle_neg;
}

double FourBarLinkageSteering::computeFloatingLinkLength(const Wheel& wheel) const
{
  // We do a little dirty trick here: when the wheel is on the right side, we assume that the geometry is mirrored:
  const double hinge_position_sign = (wheel.hinge_position_y_ < 0.0) ? -1.0 : 1.0;
  const double steering_crank_angle = config_.steering_crank_angle * hinge_position_sign;
  const double wheel_steering_arm_angle = config_.wheel_steering_arm_angle * hinge_position_sign;

  const double x_c = config_.steering_shaft_x + config_.steering_crank_length * std::cos(steering_crank_angle);
  const double y_c = config_.steering_shaft_y + config_.steering_crank_length * std::sin(steering_crank_angle);
  const double x_a = wheel.position_x_ + config_.wheel_steering_arm_length * std::cos(wheel_steering_arm_angle);
  const double y_a = wheel.hinge_position_y_ + config_.wheel_steering_arm_length * std::sin(wheel_steering_arm_angle);

  return std::hypot(x_a - x_c, y_a - y_c);
}

}
