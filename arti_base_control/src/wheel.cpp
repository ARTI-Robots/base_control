#include <arti_base_control/wheel.h>
#include <angles/angles.h>
#include <arti_base_control/steering.h>
#include <arti_base_control/utils.h>
#include <arti_base_control/vehicle.h>
#include <cmath>
#include <stdexcept>

namespace arti_base_control
{
Wheel::Wheel(const double position_x, const double position_y, const double hinge_position_y, const double radius)
  : position_x_(position_x), position_y_(position_y), hinge_position_y_(hinge_position_y), radius_(radius)
{
}

double Wheel::computeIdealWheelSteeringAngle(const double axle_steering_angle, const double icr_x) const
{
  // In case the wheel is on the ICR line (which makes no sense for a steered wheel), default to no steering:
  if (position_x_ == icr_x)
  {
    return 0.0;
  }

  const double sin_axle_steering_angle = std::sin(axle_steering_angle);
  const double x = position_x_ - icr_x;
  return normalizeSteeringAngle(
    std::atan2(x * sin_axle_steering_angle,
               x * std::cos(axle_steering_angle) - hinge_position_y_ * sin_axle_steering_angle));
}

double Wheel::computeWheelVelocity(
  const double linear_velocity, const double angular_velocity, const JointState& wheel_steering_state) const
{
  if (position_x_ == 0.0)
  {
    // If the wheel has no offset to the base link, the formula becomes very simple:
    return (linear_velocity - angular_velocity * position_y_) / radius_;
  }

  const double tangential_velocity
    = std::hypot(linear_velocity - angular_velocity * hinge_position_y_, angular_velocity * position_x_)
      * (linear_velocity < 0.0 ? -1.0 : 1.0)
      + (angular_velocity + wheel_steering_state.velocity) * (hinge_position_y_ - position_y_);

  return tangential_velocity / radius_;
}

void Wheel::computeVehicleVelocityConstraints(
  const boost::optional<JointState>& wheel_state, const JointState& wheel_steering_state,
  VehicleVelocityConstraints& constraints) const
{
  const double sin_sa = std::sin(wheel_steering_state.position);
  const double cos_sa = std::cos(wheel_steering_state.position);

  VehicleVelocityConstraint normal_velocity_constraint;
  normal_velocity_constraint.a_v_x = -sin_sa;
  normal_velocity_constraint.a_v_y = cos_sa;
  normal_velocity_constraint.a_v_theta = sin_sa * hinge_position_y_ + cos_sa * position_x_;
  normal_velocity_constraint.b = 0.0;
  constraints.push_back(normal_velocity_constraint);

  if (wheel_state)
  {
    const double corrected_tangential_velocity
      = wheel_state->velocity * radius_ - wheel_steering_state.velocity * (hinge_position_y_ - position_y_);

    VehicleVelocityConstraint tangential_velocity_constraint;
    tangential_velocity_constraint.a_v_x = cos_sa;
    tangential_velocity_constraint.a_v_y = sin_sa;
    tangential_velocity_constraint.a_v_theta
      = -cos_sa * hinge_position_y_ + sin_sa * position_x_ - position_y_ + hinge_position_y_;
    tangential_velocity_constraint.b = corrected_tangential_velocity;
    constraints.push_back(tangential_velocity_constraint);
  }
}
}
