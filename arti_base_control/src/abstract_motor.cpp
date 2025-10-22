#include <arti_base_control/abstract_motor.h>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/video/tracking.hpp>

namespace arti_base_control
{

AbstractMotor::AbstractMotor() : predication_time_out_(rclcpp::Duration::from_seconds(0.11))
{
  RCLCPP_DEBUG(rclcpp::get_logger("ArtiBaseControl"), "VescMotor::VescMotor::1");

  constexpr unsigned int state_size = 2; 
  constexpr unsigned int meas_size = 1;
  constexpr unsigned int contr_size = 0; 
  RCLCPP_DEBUG(rclcpp::get_logger("ArtiBaseControl"), "VescMotor::VescMotor::2");

  state_estimation_filter_ = cv::KalmanFilter(state_size, meas_size, contr_size, CV_32F);
  RCLCPP_DEBUG(rclcpp::get_logger("ArtiBaseControl"), "VescMotor::VescMotor::3");

  state_estimation_filter_.statePost.at<float>(0) = 0;
  state_estimation_filter_.statePost.at<float>(1) = 0;
  RCLCPP_DEBUG(rclcpp::get_logger("ArtiBaseControl"), "VescMotor::VescMotor::4");

  cv::setIdentity(state_estimation_filter_.transitionMatrix);
  RCLCPP_DEBUG(rclcpp::get_logger("ArtiBaseControl"), "VescMotor::VescMotor::5");

  state_estimation_filter_.measurementMatrix.at<float>(0) = 1.0f;
  state_estimation_filter_.measurementMatrix.at<float>(1) = 0.0f;
  RCLCPP_DEBUG(rclcpp::get_logger("ArtiBaseControl"), "VescMotor::VescMotor::6");

  state_estimation_filter_.processNoiseCov.at<float>(0, 0) = 1e-2f;
  state_estimation_filter_.processNoiseCov.at<float>(1, 1) = 1.0f;
  RCLCPP_DEBUG(rclcpp::get_logger("ArtiBaseControl"), "VescMotor::VescMotor::7");

  cv::setIdentity(state_estimation_filter_.measurementNoiseCov, 1e-1);
  RCLCPP_DEBUG(rclcpp::get_logger("ArtiBaseControl"), "VescMotor::VescMotor::8");

  cv::setIdentity(state_estimation_filter_.errorCovPre);
  RCLCPP_DEBUG(rclcpp::get_logger("ArtiBaseControl"), "VescMotor::VescMotor::9");
}

double AbstractMotor::getEstimateAt(const rclcpp::Time& time, size_t index)
{
  if (index >= 2)
  {
    throw std::runtime_error("estimate at index higher than 2 is not possible");
  }

  std::unique_lock<std::mutex> state_lock(state_mutex_);
  predict(time);
  return state_estimation_filter_.statePre.at<float>(index);
}

void AbstractMotor::correct(double estimate, bool is_mockup)
{
  std::unique_lock<std::mutex> state_lock(state_mutex_);

  RCLCPP_DEBUG(rclcpp::get_logger("ArtiBaseControl"), "VescMotor::correct: estimate: %.3f", estimate);

  rclcpp::Time now = rclcpp::Clock().now();
  last_correction_time_ = now;
  if (predict(now))
  {
    RCLCPP_DEBUG(rclcpp::get_logger("ArtiBaseControl"), "VescMotor::correct::3");
    correct(estimate);
  }
  else if (!is_mockup)
  {
    RCLCPP_WARN(rclcpp::get_logger("ArtiBaseControl"), "Skipping state correction due to failed prediction");
  }

  RCLCPP_DEBUG(rclcpp::get_logger("ArtiBaseControl"),
               "VescMotor::correct: corrected estimate: %.3f",
               state_estimation_filter_.statePost.at<float>(0));
}

void AbstractMotor::updateFilterParamets(double process_noise_0, double process_noise_1, double measurement_noise)
{
  std::unique_lock<std::mutex> state_lock(state_mutex_);

  state_estimation_filter_.processNoiseCov.at<float>(0, 0) = process_noise_0;
  state_estimation_filter_.processNoiseCov.at<float>(1, 1) = process_noise_1;
  RCLCPP_DEBUG(rclcpp::get_logger("ArtiBaseControl"), "Updated process noise parameters");

  cv::setIdentity(state_estimation_filter_.measurementNoiseCov, measurement_noise);
}

bool AbstractMotor::predict(const rclcpp::Time& time)
{
  RCLCPP_DEBUG(rclcpp::get_logger("ArtiBaseControl"), "VescMotor::predict::1");

  if (time > last_prediction_time_)
  {
    RCLCPP_DEBUG(rclcpp::get_logger("ArtiBaseControl"), "VescMotor::predict::2");

    if (!last_prediction_time_.nanoseconds() == 0)
    {
      RCLCPP_DEBUG(rclcpp::get_logger("ArtiBaseControl"), "VescMotor::predict::3");

      if (!last_correction_time_.nanoseconds() == 0 &&
          ((time > last_correction_time_) && ((time - last_correction_time_) > predication_time_out_)))
      {
        RCLCPP_WARN(rclcpp::get_logger("ArtiBaseControl"),
                    "No state update received in %.3f seconds, will not perform prediction",
                    (time - last_correction_time_).seconds());
        return false;
      }

      const double dt = (time - last_prediction_time_).seconds();
      state_estimation_filter_.transitionMatrix.at<float>(0, 1) = static_cast<float>(dt);
      state_estimation_filter_.predict();
    }

    RCLCPP_DEBUG(rclcpp::get_logger("ArtiBaseControl"), "VescMotor::predict::4");

    last_prediction_time_ = time;
    return true;
  }

  RCLCPP_DEBUG(rclcpp::get_logger("ArtiBaseControl"), "VescMotor::predict::5");
  return false;
}

void AbstractMotor::correct(double estimate)
{
  RCLCPP_DEBUG(rclcpp::get_logger("ArtiBaseControl"), "VescMotor::correct::1 estimate: %.3f", estimate);

  const cv::Vec<float, 1> measurement(static_cast<float>(estimate));
  state_estimation_filter_.correct(cv::Mat(measurement, false));

  RCLCPP_DEBUG(rclcpp::get_logger("ArtiBaseControl"), "VescMotor::correct::2");
}

}  // namespace arti_base_control
