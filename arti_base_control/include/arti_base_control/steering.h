#ifndef ARTI_BASE_CONTROL_STEERING_H
#define ARTI_BASE_CONTROL_STEERING_H

#include <arti_base_control/FourBarLinkageSteeringConfig.h>
#include <arti_base_control/IdealAckermannSteeringConfig.h>
#include <arti_base_control/joint_limits.h>
#include <arti_base_control/joint_state.h>
#include <arti_base_control/types.h>
#include <dynamic_reconfigure/server.h>
#include <ros/node_handle.h>

namespace arti_base_control
{
/**
 * Representation of the steering geometry for one or more wheels.
 *
 * Instances are used to compute the wheel's steering angle and angular velocity from a steering position (e.g.
 * steering shaft angle) and velocity, and the other way around.
 */
class Steering
{
public:
  virtual ~Steering() = default;

  /**
   * Computes a wheel's steering angle and velocity from a steering position and velocity.
   *
   * \param wheel the wheel geometry.
   * \param steering_state the steering position (e.g. steering shaft angle) and its change over time.
   * \return the steering angle (rotation in the ground plane) of the wheel, and its change over time.
   */
  virtual JointState computeWheelSteeringState(const Wheel& wheel, const JointState& steering_state) const = 0;

  /**
   * Computes the steering position from a wheel's steering angle.
   *
   * \param wheel the wheel geometry.
   * \param wheel_steering_angle the steering angle (rotation in the ground plane) of the wheel.
   * \return the steering position (e.g. steering shaft angle).
   */
  virtual double computeSteeringPosition(const Wheel& wheel, double wheel_steering_angle) const = 0;

  /**
   * Computes a wheel's steering angle limits.
   *
   * \param wheel the wheel geometry.
   * \return the steering angle limits (the minimum and maximum wheel steering angles).
   */
  virtual JointLimits computeWheelSteeringLimits(const Wheel& wheel) const = 0;

  /**
   * Adds states of joints corresponding to this steering to the given joint states.
   *
   * \param steering_state the steering position (e.g. steering shaft angle) and its change over time.
   * \param joint_states the container to add joint states to.
   */
  virtual void getJointStates(const JointState& steering_state, JointStates& joint_states) const = 0;
};

/**
 * Computes the steering angle of the wheel from the steering angle of the hypothetical central wheel, assuming ideal
 * Ackermann geometry.
 */
class IdealAckermannSteering : public Steering
{
public:
  explicit IdealAckermannSteering(const ros::NodeHandle& nh);

  JointState computeWheelSteeringState(const Wheel& wheel, const JointState& steering_state) const override;

  double computeSteeringPosition(const Wheel& wheel, double wheel_steering_angle) const override;

  JointLimits computeWheelSteeringLimits(const Wheel& wheel) const override;

  void getJointStates(const JointState& steering_state, JointStates& joint_states) const override;

protected:
  void reconfigure(IdealAckermannSteeringConfig& config);

  IdealAckermannSteeringConfig config_;
  dynamic_reconfigure::Server<IdealAckermannSteeringConfig> config_server_;
};

/**
 * Computes the steering angle of the wheel from the angle of the steering shaft, assuming a planar four-bar linkage
 * mechanism.
 */
class FourBarLinkageSteering : public Steering
{
public:
  explicit FourBarLinkageSteering(const ros::NodeHandle& nh);

  JointState computeWheelSteeringState(const Wheel& wheel, const JointState& steering_state) const override;

  double computeSteeringPosition(const Wheel& wheel, double wheel_steering_angle) const override;

  JointLimits computeWheelSteeringLimits(const Wheel& wheel) const override;

  void getJointStates(const JointState& steering_state, JointStates& joint_states) const override;

protected:
  void reconfigure(FourBarLinkageSteeringConfig& config);
  double computeWheelSteeringAngle(const Wheel& wheel, double steering_position) const;
  double computeFloatingLinkLength(const Wheel& wheel) const;

  FourBarLinkageSteeringConfig config_;
  dynamic_reconfigure::Server<FourBarLinkageSteeringConfig> config_server_;
};
}

#endif //ARTI_BASE_CONTROL_STEERING_H
