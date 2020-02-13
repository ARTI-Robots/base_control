#include <arti_base_control_vesc/vesc_drive_motor.h>
#include <vesc_driver/motor_controller_state.h>
#include <vesc_driver/vesc_driver_interface.h>
#include <functional>
#include <vesc_driver/vesc_driver_mockup.h>

namespace arti_base_control_vesc
{
VescDriveMotor::VescDriveMotor(
  const ros::NodeHandle& private_nh, const vesc_driver::DriverFactoryPtr& driver_factory,
  const std::chrono::duration<double>& execution_duration)
  : VescMotor(private_nh, driver_factory, execution_duration), reconfigure_server_(private_nh)
{
  ROS_DEBUG_STREAM("VescDriveMotor::VescDriveMotor::1");

  reconfigure_server_.setCallback(std::bind(&VescDriveMotor::reconfigure, this, std::placeholders::_1));

  ROS_DEBUG_STREAM("VescDriveMotor::VescDriveMotor::2");
}

double VescDriveMotor::getVelocity(const ros::Time& time)
{
  return getEstimateAt(time, 0);
}

void VescDriveMotor::setVelocity(double velocity)
{
  std::unique_lock<std::mutex> config_lock(config_mutex_);
  driver_->setSpeed(velocity * getVelocityConversionFactor());
  //if (private_nh_.getNamespace().rfind("/front_axle/left_motor") != std::string::npos)
  //{
  //  ROS_INFO_STREAM("VescDriveMotor::setVelocity: " << private_nh_.getNamespace() << " -- velocity: " << velocity);
  //}
}

void VescDriveMotor::brake(double current)
{
  std::unique_lock<std::mutex> config_lock(config_mutex_);
  driver_->setBrake(current);
}

void VescDriveMotor::reconfigure(DriveMotorConfig& config)
{
  ROS_DEBUG_STREAM("VescDriveMotor::reconfigure::1");

  std::unique_lock<std::mutex> config_lock(config_mutex_);

  ROS_DEBUG_STREAM("VescDriveMotor::reconfigure::1");

  config_ = config;

  ROS_DEBUG_STREAM("VescDriveMotor::reconfigure::2");

  if (config_.motor_poles == 0.0)
  {
    ROS_ERROR("Parameter motor_poles is not set");
  }

  if (!driver_)
  {
    createDriver();
  }

  if (driver_->isMockup())
  {
    const auto casted_driver = std::dynamic_pointer_cast<vesc_driver::VescDriverMockup>(driver_);

    if (!casted_driver)
    {
      ROS_ERROR("Mockup can not be casted to mockup class");
    }
    else
    {
      casted_driver->setMaxCurrent(config_.mockup_max_current);
      casted_driver->setCurrentToAcceleration(config_.mockup_current_to_acceleration);
    }
  }

  updateFilterParamets(config_.process_noise_v, config_.process_noise_a, config_.measurement_noise);
}

void VescDriveMotor::processMotorControllerState(const vesc_driver::MotorControllerState& state)
{
  std::unique_lock<std::mutex> config_lock(config_mutex_);
  double velocity = state.speed / getVelocityConversionFactor();

  correct(velocity, driver_->isMockup());
}

double VescDriveMotor::getVelocityConversionFactor() const
{
  // The Vesc controller expects and reports velocities in electrical RPM:
  return (config_.invert_direction ? -1.0 : 1.0) * config_.motor_poles * config_.velocity_correction * 30.0 * M_1_PI;
}

}
