#ifndef ARTI_BASE_CONTROL_JOINT_STATE_H
#define ARTI_BASE_CONTROL_JOINT_STATE_H

namespace arti_base_control
{
struct JointState
{
  JointState() = default;
  JointState(double position_, double velocity_);

  double position = 0.0;
  double velocity = 0.0;
};
}

#endif //ARTI_BASE_CONTROL_JOINT_STATE_H
