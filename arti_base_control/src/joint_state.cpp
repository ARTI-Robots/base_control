#include <arti_base_control/joint_state.h>

namespace arti_base_control
{
JointState::JointState(double position_, double velocity_)
  : position(position_), velocity(velocity_)
{
}
}
