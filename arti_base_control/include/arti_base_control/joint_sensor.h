#ifndef ARTI_BASE_CONTROL_JOINT_SENSOR_H
#define ARTI_BASE_CONTROL_JOINT_SENSOR_H

#include <arti_base_control/joint_state.h>
#include <arti_base_control/types.h>
#include <boost/optional.hpp>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/time.h>

namespace arti_base_control
{
class JointSensor
{
public:
  virtual ~JointSensor() = default;

  /**
   * Determines the state of the joint as stored or estimated at the given time.
   *
   * \param time the time of the joint's state.
   * \return the state of the joint at the given time.
   */
  virtual JointState getState(const ros::Time& time) = 0;

  /**
   * Determines the current supply voltage.
   *
   * \return the current supply voltage, if supported, otherwise boost::none.
   */
  virtual boost::optional<double> getSupplyVoltage() = 0;
};

class PublishingJointSensor : public JointSensor
{
public:
  PublishingJointSensor(ros::NodeHandle& node_handle, const JointSensorPtr& joint_sensor);

  JointState getState(const ros::Time& time) override;

  boost::optional<double> getSupplyVoltage() override;

protected:
  JointSensorPtr joint_sensor_;
  ros::Publisher position_publisher_;
  ros::Publisher velocity_publisher_;
};
}

#endif //ARTI_BASE_CONTROL_JOINT_SENSOR_H
