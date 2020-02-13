#include <arti_base_control_vesc/vesc_motor.h>
#include <vesc_driver/motor_controller_state.h>
#include <vesc_driver/driver_factory.h>
#include <limits>

namespace arti_base_control_vesc
{
VescMotor::VescMotor(const ros::NodeHandle& private_nh, const vesc_driver::DriverFactoryPtr& driver_factory,
                     const std::chrono::duration<double>& execution_duration)
  : private_nh_(private_nh), driver_factory_(driver_factory), execution_duration_(execution_duration),
    supply_voltage_(std::numeric_limits<double>::quiet_NaN())
{
  ROS_DEBUG_STREAM("VescMotor::VescMotor::1");

  // Init Kalman filter:
  unsigned int state_size = 2; // [v, a]
  unsigned int meas_size = 1; // [v]
  unsigned int contr_size = 0; // []
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

double VescMotor::getSupplyVoltage()
{
  return supply_voltage_;
}

void VescMotor::createDriver()
{
  driver_ = driver_factory_->createDriver(
    private_nh_, execution_duration_,
    std::bind(&VescMotor::callProcessMotorControllerState, this, std::placeholders::_1));
}

void VescMotor::callProcessMotorControllerState(const vesc_driver::MotorControllerState& state)
{
  // This indirection must be done because std::bind in the constructor doesn't work right with virtual functions.
  processMotorControllerState(state);
  supply_voltage_ = state.voltage_input;
}

void VescMotor::updateFilterParamets(double process_noise_0, double process_noise_1, double measurement_noise)
{
  std::unique_lock<std::mutex> state_lock(state_mutex_);

  state_estimation_filter_.processNoiseCov.at<float>(0, 0) = process_noise_0;
  state_estimation_filter_.processNoiseCov.at<float>(1, 1) = process_noise_1;
  ROS_DEBUG_STREAM("VescMotor::VescMotor::7");

  // Measurement noise covariance matrix (R):
  cv::setIdentity(state_estimation_filter_.measurementNoiseCov, measurement_noise);
}

double VescMotor::getEstimateAt(const ros::Time& time, size_t index)
{
  if (index >= 2)
  {
    throw std::runtime_error("estimate at index high then 2 is not possible");
  }

  std::unique_lock<std::mutex> state_lock(state_mutex_);
  predict(time);
  return state_estimation_filter_.statePre.at<float>(index);
}

bool VescMotor::predict(const ros::Time& time)
{
  ROS_DEBUG_STREAM("VescMotor::predict::1");

  if (time > last_prediction_time_)
  {
    ROS_DEBUG_STREAM("VescMotor::predict::2");

    if (!last_prediction_time_.isZero())
    {
      ROS_DEBUG_STREAM("VescMotor::predict::3");

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
void VescMotor::correct(double estimate, bool is_mockup)
{
  std::unique_lock<std::mutex> state_lock(state_mutex_);

  ROS_DEBUG_STREAM("VescMotor::correct: estimate: " << estimate);

  ros::Time now = ros::Time::now();
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

void VescMotor::correct(double estimate)
{
  ROS_DEBUG_STREAM("VescMotor::correct::1 estimate: " << estimate);

  // Kalman Correction
  const cv::Vec<float, 1> measurement(static_cast<float>(estimate));
  state_estimation_filter_.correct(cv::Mat(measurement, false));

  ROS_DEBUG_STREAM("VescMotor::correct::2");
}
}
