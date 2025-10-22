#ifndef ARTI_BASE_CONTROL_VEHICLE_CONFIG_H
#define ARTI_BASE_CONTROL_VEHICLE_CONFIG_H

#include <cmath>

namespace arti_base_control
{
struct VehicleConfig
{
  double max_velocity_linear = 1.0;
  double max_velocity_angular = M_PI * 2.0;
  double max_steering_angle = M_PI_2;
  double wheelbase = 0.0;
  double icr_x = 0.0;
  double allowed_brake_velocity = 0.0;
  double brake_velocity = 0.0;
  double brake_current = 0.0;
};
}

#endif
