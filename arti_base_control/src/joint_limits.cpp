#include <arti_base_control/joint_limits.h>

namespace arti_base_control
{
JointLimits::JointLimits(double min_position_, double max_position_)
  : min_position(min_position_), max_position(max_position_)
{
}
}
