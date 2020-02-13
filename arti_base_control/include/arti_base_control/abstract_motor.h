#ifndef ARTI_BASE_CONTROL_ABSTRACT_MOTOR_H
#define ARTI_BASE_CONTROL_ABSTRACT_MOTOR_H

#include <opencv2/video/tracking.hpp>
#include <mutex>
#include <ros/ros.h>

namespace arti_base_control
{
class AbstractMotor
{
protected:
  AbstractMotor();

  double getEstimateAt(const ros::Time& time, size_t index);
  void correct(double estimate, bool is_mockup);
  void updateFilterParamets(double process_noise_0, double process_noise_1, double measurement_noise);

private:

  bool predict(const ros::Time &time);
  void correct(double estimate);

  std::mutex state_mutex_;
  cv::KalmanFilter state_estimation_filter_;
  ros::Time last_prediction_time_;
};
}

#endif //ARTI_BASE_CONTROL_ABSTRACT_MOTOR_H
