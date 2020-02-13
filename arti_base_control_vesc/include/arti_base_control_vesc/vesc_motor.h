#ifndef ARTI_BASE_CONTROL_VESC_VESCMOTOR_H
#define ARTI_BASE_CONTROL_VESC_VESCMOTOR_H

#include <atomic>
#include <chrono>
#include <ros/node_handle.h>
#include <vesc_driver/types.h>
#include <opencv2/video/tracking.hpp>
#include <mutex>

namespace arti_base_control_vesc
{
class VescMotor
{
public:
  VescMotor(const ros::NodeHandle& private_nh, const vesc_driver::DriverFactoryPtr& driver_factory,
            const std::chrono::duration<double>& execution_duration);
  virtual ~VescMotor() = default;

  /**
   * Gets the motor controller's supply voltage in V.
   * @return the supply voltage in V.
   */
  double getSupplyVoltage();

protected:
  void createDriver();
  virtual void processMotorControllerState(const vesc_driver::MotorControllerState& state) = 0;

  ros::NodeHandle private_nh_;
  vesc_driver::VescDriverInterfacePtr driver_;

  double getEstimateAt(const ros::Time& time, size_t index);
  void correct(double estimate, bool is_mockup);
  void updateFilterParamets(double process_noise_0, double process_noise_1, double measurement_noise);

private:
  void callProcessMotorControllerState(const vesc_driver::MotorControllerState& state);

  bool predict(const ros::Time &time);
  void correct(double estimate);

  vesc_driver::DriverFactoryPtr driver_factory_;
  std::chrono::duration<double> execution_duration_;
  std::atomic<double> supply_voltage_;

  std::mutex state_mutex_;
  cv::KalmanFilter state_estimation_filter_;
  ros::Time last_prediction_time_;
};

typedef std::shared_ptr<VescMotor> VescMotorPtr;
}

#endif //ARTI_BASE_CONTROL_VESC_VESCMOTOR_H
