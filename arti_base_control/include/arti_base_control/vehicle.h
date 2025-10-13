#ifndef ARTI_BASE_CONTROL_VEHICLE_H
#define ARTI_BASE_CONTROL_VEHICLE_H

#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include <arti_base_control/axle.h>
#include <arti_base_control/types.h>
#include <arti_base_control/VehicleConfig.h>
#include <boost/optional.hpp>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/msg/twist.hpp>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <vector>

namespace arti_base_control
{
struct VehicleVelocityConstraint
{
  VehicleVelocityConstraint() = default;
  VehicleVelocityConstraint(double a_v_x_, double a_v_y_, double a_v_theta_, double b_);

  double a_v_x = 0.0;
  double a_v_y = 0.0;
  double a_v_theta = 0.0;
  double b = 0.0;
};

struct VehicleState
{
  std::vector<AxleState> axle_states;
};

class Vehicle
{
public:
  Vehicle(const rclcpp::Node& nh, const JointActuatorFactoryPtr& motor_factory, bool process_ackermann);

  void setVelocity(const ackermann_msgs::msg::AckermannDrive& velocity, const rclcpp::Time& time);
  void setVelocity(const geometry_msgs::msg::Twist& velocity, const rclcpp::Time& time);

  VehicleState getState(const rclcpp::Time& time) const;

  void getVelocity(const VehicleState& state, geometry_msgs::msg::Twist& velocity) const;
  void getVelocity(const VehicleState& state, ackermann_msgs::msg::AckermannDrive& velocity) const;
  void getJointStates(const VehicleState& state, JointStates& joint_states) const;

  boost::optional<double> getSupplyVoltage();

protected:
  void reconfigure(VehicleConfig& config);
  static double limit(double value, double max, const char* name);

  rclcpp::Node nh_;
  JointActuatorFactoryPtr motor_factory_;
  VehicleConfig config_;
  dynamic_reconfigure::Server<VehicleConfig> reconfigure_server_;
  std::vector<AxlePtr> axles_;

  double wheelbase_ = 0.0;
  bool process_ackermann_;
};
}

#endif //ARTI_BASE_CONTROL_VEHICLE_H
