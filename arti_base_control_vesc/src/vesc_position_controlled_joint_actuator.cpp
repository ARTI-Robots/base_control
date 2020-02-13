#include <arti_base_control_vesc/vesc_position_controlled_joint_actuator.h>
#include <functional>
#include <vesc_driver/motor_controller_state.h>
#include <vesc_driver/vesc_driver_interface.h>
#include <vesc_driver/vesc_driver_mockup.h>

namespace arti_base_control_vesc
{
VescPositionControlledJointActuator::VescPositionControlledJointActuator(
  ros::NodeHandle& private_nh, const vesc_driver::DriverFactoryPtr& driver_factory, double control_interval)
  : VescMotor(private_nh, driver_factory, std::chrono::duration<double>(control_interval)),
    reconfigure_server_(private_nh)
{
  reconfigure_server_.setCallback(
    std::bind(&VescPositionControlledJointActuator::reconfigure, this, std::placeholders::_1));
}

arti_base_control::JointState VescPositionControlledJointActuator::getState(const ros::Time& time)
{
  return arti_base_control::JointState(getEstimateAt(time, 0), getEstimateAt(time, 1));
}

void VescPositionControlledJointActuator::setPosition(double position)
{
  std::unique_lock<std::mutex> config_lock(config_mutex_);
  const double position_in_rad = position * (config_.invert_direction ? -1. : 1.) + config_.position_offset;
  const double position_in_deg = position_in_rad / M_PI * 180.0;
  ROS_DEBUG_STREAM("VescSteeringMotor::setPosition: this: " << this << ", position_in_deg: " << position_in_deg);
  driver_->setPosition(position_in_deg);
}

boost::optional<double> VescPositionControlledJointActuator::getSupplyVoltage()
{
  return getMotorSupplyVoltage();
}

void VescPositionControlledJointActuator::reconfigure(SteeringMotorConfig& config)
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

void VescPositionControlledJointActuator::processMotorControllerState(const vesc_driver::MotorControllerState& state)
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
