#ifndef ARTI_BASE_CONTROL_STEERING_H
#define ARTI_BASE_CONTROL_STEERING_H

#include <arti_base_control/types.h>
#include <arti_base_control/FourBarLinkageSteeringConfig.h>
#include <arti_base_control/IdealAckermannSteeringConfig.h>
#include <dynamic_reconfigure/server.h>
#include <ros/node_handle.h>

namespace arti_base_control
{
/**
 * Representation of the steering geometry for one or more wheels.
 *
 * Instances are used to compute the wheel's steering angle and angular velocity from a steering position (e.g.
 * steering shaft angle) and velocity.
 */
class Steering
{
public:
  virtual ~Steering() = default;

  /**
   * Computes a wheel's steering angle from a steering position.
   *
   * \param wheel the wheel geometry.
   * \param steering_position the steering position (e.g. steering shaft angle).
   * \return the steering angle (rotation in the ground plane) of the wheel.
   */
  virtual double computeWheelSteeringAngle(const Wheel& wheel, double steering_position) const = 0;

  /**
   * Computes a wheel's steering velocity from a steering position.
   *
   * \param wheel the wheel geometry.
   * \param steering_position the steering position (e.g. steering shaft angle).
   * \param steering_velocity the steering velocity (change of steering position over time).
   * \return the steering velocity (change of steering angle over time) of the wheel.
   */
  virtual double computeWheelSteeringVelocity(
    const Wheel& wheel, double steering_position, double steering_velocity) const = 0;

  /**
   * Computes the steering position from a wheel's steering angle.
   *
   * \param wheel the wheel geometry.
   * \param wheel_steering_angle the steering angle (rotation in the ground plane) of the wheel.
   * \return the steering position (e.g. steering shaft angle).
   */
  virtual double computeSteeringPosition(const Wheel& wheel, double wheel_steering_angle) const = 0;
};

/**
 * Computes the steering angle of the wheel from the steering angle of the hypothetical central wheel, assuming ideal
 * Ackermann geometry.
 */
class IdealAckermannSteering : public Steering
{
public:
  explicit IdealAckermannSteering(const ros::NodeHandle& nh);

  double computeWheelSteeringAngle(const Wheel& wheel, double steering_position) const override;

  double computeWheelSteeringVelocity(
    const Wheel& wheel, double steering_position, double steering_velocity) const override;

  double computeSteeringPosition(const Wheel& wheel, double wheel_steering_angle) const override;

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

  double computeWheelSteeringAngle(const Wheel& wheel, double steering_position) const override;

  double computeWheelSteeringVelocity(
    const Wheel& wheel, double steering_position, double steering_velocity) const override;

  double computeSteeringPosition(const Wheel& wheel, double wheel_steering_angle) const override;

protected:
  void reconfigure(FourBarLinkageSteeringConfig& config);
  double computeFloatingLinkLength(const Wheel& wheel) const;

    FourBarLinkageSteeringConfig config_;
  dynamic_reconfigure::Server<FourBarLinkageSteeringConfig> config_server_;
};
}

#endif //ARTI_BASE_CONTROL_STEERING_H
