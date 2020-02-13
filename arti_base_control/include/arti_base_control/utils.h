#ifndef ARTI_BASE_CONTROL_UTILS_H
#define ARTI_BASE_CONTROL_UTILS_H

namespace arti_base_control
{

double normalizeSteeringAngle(double steering_angle);

template<typename M>
M makeDataMsg(const typename M::_data_type& data)
{
  M msg;
  msg.data = data;
  return msg;
}

}

#endif //ARTI_BASE_CONTROL_UTILS_H
