#include <arti_base_control/joint_sensor.h>
#include <arti_base_control/utils.h>
#include <std_msgs/Float64.h>

namespace arti_base_control
{
PublishingJointSensor::PublishingJointSensor(ros::NodeHandle& node_handle, const JointSensorPtr& joint_sensor)
  : joint_sensor_(joint_sensor), position_publisher_(node_handle.advertise<std_msgs::Float64>("position", 1)),
    velocity_publisher_(node_handle.advertise<std_msgs::Float64>("velocity", 1))
{
}

JointState PublishingJointSensor::getState(const ros::Time& time)
{
  const JointState state = joint_sensor_->getState(time);
  position_publisher_.publish(makeDataMsg<std_msgs::Float64>(state.position));
  velocity_publisher_.publish(makeDataMsg<std_msgs::Float64>(state.velocity));
  return state;
}

boost::optional<double> PublishingJointSensor::getSupplyVoltage()
{
  return joint_sensor_->getSupplyVoltage();
}
}
