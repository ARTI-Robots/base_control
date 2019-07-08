#ifndef ARTI_BASE_CONTROL_AXLE_H
#define ARTI_BASE_CONTROL_AXLE_H

#include <ackermann_msgs/AckermannDrive.h>
#include <arti_base_control/AxleConfig.h>
#include <arti_base_control/joint_state.h>
#include <arti_base_control/types.h>
#include <arti_base_control/VehicleConfig.h>
#include <arti_base_control/wheel.h>
#include <boost/optional.hpp>
#include <dynamic_reconfigure/server.h>
#include <memory>
#include <ros/node_handle.h>
#include <ros/time.h>

namespace arti_base_control
{
struct AxleState
{
  boost::optional<JointState> steering_motor_state;
  boost::optional<JointState> left_motor_state;
  boost::optional<JointState> right_motor_state;
};

class Axle
{
public:
  Axle(const ros::NodeHandle& nh, const VehicleConfig& vehicle_config, const JointActuatorFactoryPtr& motor_factory);

  const AxleConfig& getConfig() const;
  void setVehicleConfig(const VehicleConfig& vehicle_config);

  void setVelocity(double linear_velocity, double angular_velocity, double axle_steering_angle, const ros::Time& time);

  AxleState getState(const ros::Time& time) const;

  void getVelocityConstraints(const AxleState& state, VehicleVelocityConstraints& constraints) const;

  void getJointStates(const AxleState& state, JointStates& joint_states) const;

  boost::optional<double> getSupplyVoltage();

protected:
  void reconfigure(AxleConfig& config);

  ros::NodeHandle nh_;
  JointActuatorFactoryPtr motor_factory_;

  VehicleConfig vehicle_config_;
  boost::optional<AxleConfig> config_;
  dynamic_reconfigure::Server<AxleConfig> reconfigure_server_;

  SteeringConstPtr steering_;
  Wheel left_wheel_;
  Wheel right_wheel_;

  PositionControlledJointActuatorPtr steering_motor_;
  VelocityControlledJointActuatorPtr left_motor_;
  VelocityControlledJointActuatorPtr right_motor_;
};
}

#endif //ARTI_BASE_CONTROL_AXLE_H
