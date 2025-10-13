#ifndef ARTI_BASE_CONTROL_ABSTRACT_MOTOR_H
#define ARTI_BASE_CONTROL_ABSTRACT_MOTOR_H

#include <opencv2/video/tracking.hpp>
#include <mutex>
#include "rclcpp/rclcpp.hpp"

namespace arti_base_control
{
class AbstractMotor
{
protected:
  AbstractMotor();

  double getEstimateAt(const rclcpp::Time& time, size_t index);
  void correct(double estimate, bool is_mockup);
  void updateFilterParamets(double process_noise_0, double process_noise_1, double measurement_noise);

private:

  bool predict(const rclcpp::Time &time);
  void correct(double estimate);

  std::mutex state_mutex_;
  cv::KalmanFilter state_estimation_filter_;
  rclcpp::Time last_prediction_time_;
  rclcpp::Time last_correction_time_;

  rclcpp::Duration predication_time_out_;
};
}

#endif //ARTI_BASE_CONTROL_ABSTRACT_MOTOR_H
