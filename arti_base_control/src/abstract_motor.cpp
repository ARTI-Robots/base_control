#include <arti_base_control/abstract_motor.h>

namespace arti_base_control
{
AbstractMotor::AbstractMotor() : predication_time_out_(0.11)
{
  ROS_DEBUG_STREAM("VescMotor::VescMotor::1");

  // Init Kalman filter:
  constexpr unsigned int state_size = 2; // [e0, e1]
  constexpr unsigned int meas_size = 1; // [e0]
  constexpr unsigned int contr_size = 0; // []
  ROS_DEBUG_STREAM("VescMotor::VescMotor::2");

  state_estimation_filter_ = cv::KalmanFilter(state_size, meas_size, contr_size, CV_32F);
  ROS_DEBUG_STREAM("VescMotor::VescMotor::3");

  // Corrected state (x(k)): x(k)=x'(k)+K(k)*(z(k)-H*x'(k)):
  // [e0, e1]
  state_estimation_filter_.statePost.at<float>(0) = 0;
  state_estimation_filter_.statePost.at<float>(1) = 0;
  ROS_DEBUG_STREAM("VescMotor::VescMotor::4");

  // State transition matrix (A):
  // [ 1 dT]
  // [ 0 1]
  // Note: set dT at each processing step!
  cv::setIdentity(state_estimation_filter_.transitionMatrix);
  ROS_DEBUG_STREAM("VescMotor::VescMotor::5");

  // Measurement matrix (H):
  // [ 1 0]
  state_estimation_filter_.measurementMatrix.at<float>(0) = 1.0f;
  state_estimation_filter_.measurementMatrix.at<float>(1) = 0.0f;
  ROS_DEBUG_STREAM("VescMotor::VescMotor::6");

  // Process noise covariance matrix (Q):
  // [ E0 0  ]
  // [ 0  E1 ]
  state_estimation_filter_.processNoiseCov.at<float>(0, 0) = 1e-2f;
  state_estimation_filter_.processNoiseCov.at<float>(1, 1) = 1.0f;
  ROS_DEBUG_STREAM("VescMotor::VescMotor::7");

  // Measurement noise covariance matrix (R):
  cv::setIdentity(state_estimation_filter_.measurementNoiseCov, 1e-1);
  ROS_DEBUG_STREAM("VescMotor::VescMotor::8");

  // Priori error estimate covariance matrix (P'(k)): P'(k)=A*P(k-1)*At + Q):
  cv::setIdentity(state_estimation_filter_.errorCovPre);
  ROS_DEBUG_STREAM("VescMotor::VescMotor::9");
}

double AbstractMotor::getEstimateAt(const ros::Time& time, size_t index)
{
  if (index >= 2)
  {
    throw std::runtime_error("estimate at index high then 2 is not possible");
  }

  std::unique_lock<std::mutex> state_lock(state_mutex_);
  predict(time);
  return state_estimation_filter_.statePre.at<float>(index);
}

void AbstractMotor::correct(double estimate, bool is_mockup)
{
  std::unique_lock<std::mutex> state_lock(state_mutex_);

  ROS_DEBUG_STREAM("VescMotor::correct: estimate: " << estimate);

  ros::Time now = ros::Time::now();
  last_correction_time_ = now;
  if (predict(now)) // only correct if prediction can be performed
  {
    ROS_DEBUG_STREAM("VescMotor::correct::3");

    correct(estimate);
  }
  else if (!is_mockup)
  {
    ROS_WARN("Skipping state correction due to failed prediction");
  }

  ROS_DEBUG_STREAM("VescMotor::correct: corrected estimate: "
                     << state_estimation_filter_.statePost.at<float>(0));
}

void AbstractMotor::updateFilterParamets(double process_noise_0, double process_noise_1, double measurement_noise)
{
  std::unique_lock<std::mutex> state_lock(state_mutex_);

  state_estimation_filter_.processNoiseCov.at<float>(0, 0) = process_noise_0;
  state_estimation_filter_.processNoiseCov.at<float>(1, 1) = process_noise_1;
  ROS_DEBUG_STREAM("VescMotor::VescMotor::7");

  // Measurement noise covariance matrix (R):
  cv::setIdentity(state_estimation_filter_.measurementNoiseCov, measurement_noise);
}

bool AbstractMotor::predict(const ros::Time& time)
{
  ROS_DEBUG_STREAM("VescMotor::predict::1");

  if (time > last_prediction_time_)
  {
    ROS_DEBUG_STREAM("VescMotor::predict::2");

    if (!last_prediction_time_.isZero())
    {
      ROS_DEBUG_STREAM("VescMotor::predict::3");

      if (!last_correction_time_.isZero()
          && ((time > last_correction_time_) && ((time - last_correction_time_) > predication_time_out_)))
      {
        ROS_WARN_STREAM("no state update received in " << (time - last_correction_time_).toSec()
                                                       << "seconds will not perform prediction");
        return false;
      }
      const double dt = (time - last_prediction_time_).toSec();
      state_estimation_filter_.transitionMatrix.at<float>(0, 1) = static_cast<float>(dt);
      state_estimation_filter_.predict();
    }

    ROS_DEBUG_STREAM("VescMotor::predict::4");

    last_prediction_time_ = time;
    return true;
  }

  ROS_DEBUG_STREAM("VescMotor::predict::5");

  return false;
}

void AbstractMotor::correct(double estimate)
{
  ROS_DEBUG_STREAM("VescMotor::correct::1 estimate: " << estimate);

  // Kalman Correction
  const cv::Vec<float, 1> measurement(static_cast<float>(estimate));
  state_estimation_filter_.correct(cv::Mat(measurement, false));

  ROS_DEBUG_STREAM("VescMotor::correct::2");
}

}
