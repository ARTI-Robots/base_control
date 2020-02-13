#ifndef ARTI_BASE_CONTROL_VESC_VESCDRIVEMOTOR_H
#define ARTI_BASE_CONTROL_VESC_VESCDRIVEMOTOR_H

#include <dynamic_reconfigure/server.h>
#include <mutex>
#include <opencv2/video/tracking.hpp>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <arti_base_control_vesc/DriveMotorConfig.h>
#include <arti_base_control_vesc/vesc_motor.h>

namespace arti_base_control_vesc
{
class VescDriveMotor : public VescMotor
{
public:
  VescDriveMotor(const ros::NodeHandle& private_nh, const vesc_driver::DriverFactoryPtr& driver_factory,
                 const std::chrono::duration<double>& execution_duration);

  /**
   * Gets the current motor velocity in rad/s, estimated at the given time.
   * @param time time at which velocity is estimated.
   * @return the estimated current velocity in rad/s.
   */
  double getVelocity(const ros::Time& time);

  /**
   * Commands the motor to revolute with the given velocity in rad/s.
   * @param velocity the velocity command in rad/s.
   */
  void setVelocity(double velocity);

  /**
   * Commands the motor to brake until it has stopped moving.
   * @param current the current (in A) to apply to the motor.
   */
  void brake(double current);

protected:
  void processMotorControllerState(const vesc_driver::MotorControllerState& state) override;

  void reconfigure(DriveMotorConfig& config);
  double getVelocityConversionFactor() const;

  bool predict(const ros::Time &time);
  void correct(double velocity);

  std::mutex config_mutex_;
  dynamic_reconfigure::Server<DriveMotorConfig> reconfigure_server_;
  DriveMotorConfig config_;

  std::mutex state_mutex_;
  cv::KalmanFilter state_estimation_filter_;
  ros::Time last_prediction_time_;
};

typedef std::shared_ptr<VescDriveMotor> VescDriveMotorPtr;
}

#endif // ARTI_BASE_CONTROL_VESC_VESCDRIVEMOTOR_H
