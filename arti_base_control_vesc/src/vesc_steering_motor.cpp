#include <arti_base_control_vesc/vesc_steering_motor.h>
#include <functional>
#include <vesc_driver/motor_controller_state.h>
#include <vesc_driver/vesc_driver_interface.h>
#include <vesc_driver/vesc_driver_mockup.h>

namespace arti_base_control_vesc
{

VescSteeringMotor::VescSteeringMotor(
  const ros::NodeHandle& private_nh, const vesc_driver::DriverFactoryPtr& driver_factory,
  const std::chrono::duration<double>& execution_duration)
  : VescMotor(private_nh, driver_factory, execution_duration), reconfigure_server_(private_nh)
{
  reconfigure_server_.setCallback(std::bind(&VescSteeringMotor::reconfigure, this, std::placeholders::_1));
}

double VescSteeringMotor::getPosition(const ros::Time& time)
{
  return getEstimateAt(time, 0);
}

double VescSteeringMotor::getVelocity(const ros::Time& time)
{
  return getEstimateAt(time, 1);
}

void VescSteeringMotor::setPosition(double position)
{
  std::unique_lock<std::mutex> config_lock(config_mutex_);
  const double position_in_rad = position * (config_.invert_direction ? -1. : 1.) + config_.position_offset;
  const double position_in_deg = position_in_rad / M_PI * 180.0;
  ROS_DEBUG_STREAM("VescSteeringMotor::setPosition: this: " << this << ", position_in_deg: " << position_in_deg);
  driver_->setPosition(position_in_deg);
  //ROS_INFO_STREAM("VescSteeringMotor::setPosition: " << private_nh_.getNamespace() << " -- position: " << position);
}

void VescSteeringMotor::reconfigure(SteeringMotorConfig& config)
{
  ROS_DEBUG_STREAM("VescSteeringMotor::reconfigure::1");

  std::unique_lock<std::mutex> config_lock(config_mutex_);

  ROS_DEBUG_STREAM("VescSteeringMotor::reconfigure::1");

  config_ = config;

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
      casted_driver->setStatePosition(config_.position_offset);
      casted_driver->setMaxCurrent(config_.mockup_max_current);
      casted_driver->setCurrentToAcceleration(config_.mockup_current_to_acceleration);
    }
  }

  updateFilterParamets(config_.process_noise_p, config_.process_noise_v, config_.measurement_noise);

  ROS_DEBUG_STREAM("VescSteeringMotor::reconfigure::2");
}

void VescSteeringMotor::processMotorControllerState(const vesc_driver::MotorControllerState& state)
{
  std::unique_lock<std::mutex> config_lock(config_mutex_);

  const double position_in_deg = state.position;
  ROS_DEBUG_STREAM("VescSteeringMotor::processMotorControllerState: this: " << this << ", position_in_deg: "
                                                                            << position_in_deg);
  const double position_in_rad = position_in_deg / 180. * M_PI;
  const double position = (position_in_rad - config_.position_offset) / (config_.invert_direction ? -1. : 1.);

  correct(position, driver_->isMockup());
}
}
