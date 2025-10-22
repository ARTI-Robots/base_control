#ifndef ARTI_BASE_CONTROL_AXLE_H
#define ARTI_BASE_CONTROL_AXLE_H

#include <ackermann_msgs/msg/ackermann_drive.hpp>
// #include <arti_base_control/AxleConfig.h>
#include <arti_base_control/joint_state.h>
#include <arti_base_control/types.h>
#include <arti_base_control/VehicleConfig.h>
#include <arti_base_control/wheel.h>
#include <boost/optional.hpp>
// #include <dynamic_reconfigure/server.h>
#include <memory>
#include <mutex>
// #include <ros/time.h>

#include <rclcpp/rclcpp.hpp>

namespace arti_base_control
{

// ----------------------
// Axle configuration struct
// ----------------------

struct AxleConfig
{
  double position_x = 0.0;
  double position_y = 0.0;
  double wheel_diameter = 0.0;
  bool is_steered = false;
  bool is_driven = false;
  double track = 0.0;
  double steering_hinge_offset = 0.0;
  double steering_velocity = 0.0;
  double steering_position_tolerance = 0.001;

  std::string left_hinge_joint;
  std::string left_wheel_joint;
  std::string right_hinge_joint;
  std::string right_wheel_joint;
};

struct AxleState
{
  boost::optional<JointState> steering_motor_state;
  boost::optional<JointState> left_motor_state;
  boost::optional<JointState> right_motor_state;
};

class Axle
{
public:
  Axle(const rclcpp::Node::SharedPtr& nh,
     const VehicleConfig& vehicle_config, 
     const JointActuatorFactoryPtr& motor_factory);

  const AxleConfig& getConfig() const;
  void setVehicleConfig(const VehicleConfig& vehicle_config);

  void setVelocity(double linear_velocity, double angular_velocity, double axle_steering_angle, const rclcpp::Time& time);

  AxleState getState(const rclcpp::Time& time) const;

  void getVelocityConstraints(const AxleState& state, VehicleVelocityConstraints& constraints) const;

  void getJointStates(const AxleState& state, JointStates& joint_states) const;

  boost::optional<double> getSupplyVoltage();

protected:
  void reconfigure(AxleConfig& config);

  rclcpp::Node::SharedPtr nh_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_cb_handle_;


  JointActuatorFactoryPtr motor_factory_;

  VehicleConfig vehicle_config_;
  boost::optional<AxleConfig> config_;

  SteeringConstPtr steering_;
  Wheel left_wheel_;
  Wheel right_wheel_;

  PositionControlledJointActuatorPtr steering_motor_;
  VelocityControlledJointActuatorPtr left_motor_;
  VelocityControlledJointActuatorPtr right_motor_;
};
}

#endif //ARTI_BASE_CONTROL_AXLE_H
