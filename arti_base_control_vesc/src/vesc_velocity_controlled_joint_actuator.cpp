#include <arti_base_control_vesc/vesc_velocity_controlled_joint_actuator.h>
#include <vesc_driver/motor_controller_state.h>
#include <vesc_driver/vesc_driver_interface.h>
#include <functional>
#include <vesc_driver/vesc_driver_mockup.h>

namespace arti_base_control_vesc
{
VescVelocityControlledJointActuator::VescVelocityControlledJointActuator(
  ros::NodeHandle& private_nh, const vesc_driver::DriverFactoryPtr& driver_factory, double control_interval)
  : VescMotor(private_nh, driver_factory, std::chrono::duration<double>(control_interval)),
    reconfigure_server_(private_nh)
{
  reconfigure_server_.setCallback(
    std::bind(&VescVelocityControlledJointActuator::reconfigure, this, std::placeholders::_1));
}

arti_base_control::JointState VescVelocityControlledJointActuator::getState(const ros::Time& time)
{
  const double velocity = getEstimateAt(time, 0);

  if (time > last_velocity_time_)
  {
    if (!last_velocity_time_.isZero())
    {
      const double time_difference = (time - last_velocity_time_).toSec();
      last_position_ += (last_velocity_ + velocity) * 0.5 * time_difference;  // Assumes constant acceleration
    }

    last_velocity_ = velocity;
    last_velocity_time_ = time;
  }

  return arti_base_control::JointState(last_position_, velocity);
}

void VescVelocityControlledJointActuator::setVelocity(const double velocity)
{
  std::unique_lock<std::mutex> config_lock(config_mutex_);
  driver_->setSpeed(velocity * getVelocityConversionFactor());
}

void VescVelocityControlledJointActuator::brake(const double current)
{
  std::unique_lock<std::mutex> config_lock(config_mutex_);
  driver_->setBrake(current);
}

boost::optional<double> VescVelocityControlledJointActuator::getSupplyVoltage()
{
  return getMotorSupplyVoltage();
}

void VescVelocityControlledJointActuator::processMotorControllerState(const vesc_driver::MotorControllerState& state)
{
  std::unique_lock<std::mutex> config_lock(config_mutex_);
  double velocity = state.speed / getVelocityConversionFactor();

  correct(velocity, driver_->isMockup());
}

void VescVelocityControlledJointActuator::reconfigure(DriveMotorConfig& config)
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

double VescVelocityControlledJointActuator::getVelocityConversionFactor() const
{
  // The Vesc controller expects and reports velocities in electrical RPM:
  return (config_.invert_direction ? -1.0 : 1.0) * config_.motor_poles * config_.velocity_correction * 30.0 * M_1_PI;
}
}
