#ifndef ARTI_BASE_CONTROL_JOINT_LIMITS_H
#define ARTI_BASE_CONTROL_JOINT_LIMITS_H

namespace arti_base_control
{
struct JointLimits
{
  JointLimits() = default;
  JointLimits(double min_position_, double max_position_);

  double min_position = 0.0;
  double max_position = 0.0;
};
}

#endif //ARTI_BASE_CONTROL_JOINT_LIMITS_H
