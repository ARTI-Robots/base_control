#ifndef ARTI_BASE_CONTROL_VESC_VESC_STEERING_MOTOR_H
#define ARTI_BASE_CONTROL_VESC_VESC_STEERING_MOTOR_H

#include <dynamic_reconfigure/server.h>
#include <mutex>
#include <opencv2/video/tracking.hpp>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <arti_base_control_vesc/SteeringMotorConfig.h>
#include <arti_base_control_vesc/vesc_motor.h>

namespace arti_base_control_vesc
{
class VescSteeringMotor : public VescMotor
{
public:
  VescSteeringMotor(const ros::NodeHandle& private_nh, const vesc_driver::DriverFactoryPtr& driver_factory,
                    const std::chrono::duration<double>& execution_duration);

  /**
   * Gets the current position of the motor in radians, estimated at the given time.
   * @param time time at which motor state is estimated.
   * @return the estimated position in radians.
   */
  double getPosition(const ros::Time& time);

  /**
   * Gets the current velocity of the motor in radians per second, estimated at the given time.
   * @param time time at which motor state is estimated.
   * @return the estimated velocity in radians per second.
   */
  double getVelocity(const ros::Time& time);

  /**
   * Commands the motor to the position in rad.
   * @param position the position command in rad.
   */
  void setPosition(double position);

private:
  void reconfigure(SteeringMotorConfig& config);
  void processMotorControllerState(const vesc_driver::MotorControllerState& state) override;

  std::mutex config_mutex_;
  dynamic_reconfigure::Server<SteeringMotorConfig> reconfigure_server_;
  SteeringMotorConfig config_;
};

typedef std::shared_ptr<VescSteeringMotor> VescSteeringMotorPtr;
}

#endif //ARTI_BASE_CONTROL_VESC_VESC_STEERING_MOTOR_H
