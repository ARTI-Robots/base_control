#include <arti_base_control/axle.h>
#include <arti_base_control/velocity_controlled_joint_actuator.h>
#include <arti_base_control/joint_actuator_factory.h>
#include <arti_base_control/steering.h>
#include <arti_base_control/position_controlled_joint_actuator.h>
#include <arti_base_control/utils.h>
#include <arti_base_control/vehicle.h>

namespace arti_base_control
{
Axle::Axle(const ros::NodeHandle& nh, const VehicleConfig& vehicle_config, const JointActuatorFactoryPtr& motor_factory)
  : nh_(nh), motor_factory_(motor_factory), vehicle_config_(vehicle_config), reconfigure_server_(nh)
{
  reconfigure_server_.setCallback(std::bind(&Axle::reconfigure, this, std::placeholders::_1));
}

void Axle::reconfigure(AxleConfig& config)
{
  if (config.wheel_diameter == 0.0)
  {
    ROS_ERROR_STREAM("Parameter wheel_diameter is zero");
  }

  if (config_)
  {
    if (config.is_steered != config_->is_steered)
    {
      ROS_ERROR_STREAM("Parameter is_steered cannot be changed dynamically");
      config.is_steered = config_->is_steered;
    }

    if (config.is_driven != config_->is_driven)
    {
      ROS_ERROR_STREAM("Parameter is_driven cannot be changed dynamically");
      config.is_driven = config_->is_driven;
    }
  }
  else
  {
    // Initialize motors when callback is called for the first time (which happens when we call setCallback):
    if (config.is_steered)
    {
      const ros::NodeHandle steering_nh(nh_, "steering");
      std::string steering_type;
      if (steering_nh.getParam("type", steering_type))
      {
        if (steering_type == "IdealAckermannSteering")
        {
          steering_.reset(new IdealAckermannSteering(steering_nh));
        }
        else if (steering_type == "FourBarLinkageSteering")
        {
          steering_.reset(new FourBarLinkageSteering(steering_nh));
        }
        else
        {
          ROS_ERROR_STREAM("Steering configuration has unknown type '" << steering_type << "'");
        }
      }
      else
      {
        ROS_ERROR_STREAM("Steering configuration lacks type");
      }

      ros::NodeHandle steering_motor_nh(nh_, "steering_motor");
      steering_motor_ = motor_factory_->createPositionControlledJointActuator(steering_motor_nh);
    }

    if (config.is_driven)
    {
      ros::NodeHandle left_motor_nh(nh_, "left_motor");
      left_motor_ = motor_factory_->createVelocityControlledJointActuator(left_motor_nh);

      ros::NodeHandle right_motor_nh(nh_, "right_motor");
      right_motor_ = motor_factory_->createVelocityControlledJointActuator(right_motor_nh);
    }
  }

  config_ = config;

  left_wheel_.position_x_ = config_->position_x;
  left_wheel_.position_y_ = config_->position_y + 0.5 * config_->track;
  left_wheel_.hinge_position_y_ = left_wheel_.position_y_ - config_->steering_hinge_offset;
  left_wheel_.radius_ = 0.5 * config_->wheel_diameter;

  right_wheel_.position_x_ = config_->position_x;
  right_wheel_.position_y_ = config_->position_y - 0.5 * config_->track;
  right_wheel_.hinge_position_y_ = right_wheel_.position_y_ + config_->steering_hinge_offset;
  right_wheel_.radius_ = 0.5 * config_->wheel_diameter;
}

const AxleConfig& Axle::getConfig() const
{
  if (config_)
  {
    return *config_;
  }
  throw std::logic_error("tried to get axle config before setting it");
}

void Axle::setVehicleConfig(const VehicleConfig& vehicle_config)
{
  vehicle_config_ = vehicle_config;
}

void Axle::setVelocity(
  const double linear_velocity, const double angular_velocity, const double axle_steering_angle, const ros::Time& time)
{
  JointState expected_steering_state;

  if (steering_motor_ && steering_)
  {
    const double left_wheel_steering_angle
      = left_wheel_.computeIdealWheelSteeringAngle(axle_steering_angle, vehicle_config_.icr_x);

    const double steering_position_from_left_wheel
      = steering_->computeSteeringPosition(left_wheel_, left_wheel_steering_angle);

    const double right_wheel_steering_angle
      = right_wheel_.computeIdealWheelSteeringAngle(axle_steering_angle, vehicle_config_.icr_x);

    const double steering_position_from_right_wheel
      = steering_->computeSteeringPosition(right_wheel_, right_wheel_steering_angle);

    const double steering_position = 0.5 * (steering_position_from_left_wheel + steering_position_from_right_wheel);

    const JointState current_steering_state = steering_motor_->getState(time);
    expected_steering_state.position = current_steering_state.position;

    const double steering_position_difference = steering_position - expected_steering_state.position;
    if (steering_position_difference > config_->steering_position_tolerance)
    {
      expected_steering_state.velocity = config_->steering_velocity;
    }
    else if (steering_position_difference < -config_->steering_position_tolerance)
    {
      expected_steering_state.velocity = -config_->steering_velocity;
    }

    steering_motor_->setPosition(steering_position);
  }

  if (left_motor_ && right_motor_)
  {
    const JointState left_wheel_steering_state
      = steering_ ? steering_->computeWheelSteeringState(left_wheel_, expected_steering_state) : JointState();
    const double left_velocity
      = left_wheel_.computeWheelVelocity(linear_velocity, angular_velocity, left_wheel_steering_state);

    const JointState right_wheel_steering_state
      = steering_ ? steering_->computeWheelSteeringState(right_wheel_, expected_steering_state) : JointState();
    const double right_velocity
      = right_wheel_.computeWheelVelocity(linear_velocity, angular_velocity, right_wheel_steering_state);

    if ((std::fabs(left_velocity) <= vehicle_config_.brake_velocity)
        && (std::fabs(right_velocity) <= vehicle_config_.brake_velocity)
        && (std::fabs(left_motor_->getState(time).velocity) <= vehicle_config_.allowed_brake_velocity)
        && (std::fabs(right_motor_->getState(time).velocity) <= vehicle_config_.allowed_brake_velocity))
    {
      left_motor_->brake(vehicle_config_.brake_current);
      right_motor_->brake(vehicle_config_.brake_current);
    }
    else
    {
      left_motor_->setVelocity(left_velocity);
      right_motor_->setVelocity(right_velocity);
    }
  }
}

AxleState Axle::getState(const ros::Time& time) const
{
  AxleState state;
  if (steering_motor_)
  {
    state.steering_motor_state.emplace(steering_motor_->getState(time));
  }

  if (left_motor_)
  {
    state.left_motor_state.emplace(left_motor_->getState(time));
  }

  if (right_motor_)
  {
    state.right_motor_state.emplace(right_motor_->getState(time));
  }
  return state;
}

void Axle::getVelocityConstraints(const AxleState& state, VehicleVelocityConstraints& constraints) const
{
  const JointState steering_motor_state = state.steering_motor_state.get_value_or(JointState(0.0, 0.0));
  const JointState left_wheel_steering_state
    = steering_ ? steering_->computeWheelSteeringState(left_wheel_, steering_motor_state) : JointState();
  const JointState right_wheel_steering_state
    = steering_ ? steering_->computeWheelSteeringState(right_wheel_, steering_motor_state) : JointState();

  left_wheel_.computeVehicleVelocityConstraints(state.left_motor_state, left_wheel_steering_state, constraints);
  right_wheel_.computeVehicleVelocityConstraints(state.right_motor_state, right_wheel_steering_state, constraints);
}

void Axle::getJointStates(const AxleState& state, JointStates& joint_states) const
{
  if (config_)
  {
    if (state.left_motor_state && !config_->left_wheel_joint.empty())
    {
      joint_states[config_->left_wheel_joint] = *state.left_motor_state;
    }

    if (state.right_motor_state && !config_->right_wheel_joint.empty())
    {
      joint_states[config_->right_wheel_joint] = *state.right_motor_state;
    }

    if (state.steering_motor_state && steering_)
    {
      steering_->getJointStates(*state.steering_motor_state, joint_states);

      if (!config_->left_hinge_joint.empty())
      {
        joint_states[config_->left_hinge_joint]
          = steering_->computeWheelSteeringState(left_wheel_, *state.steering_motor_state);
      }

      if (!config_->right_hinge_joint.empty())
      {
        joint_states[config_->right_hinge_joint]
          = steering_->computeWheelSteeringState(right_wheel_, *state.steering_motor_state);
      }
    }
  }
}

boost::optional<double> Axle::getSupplyVoltage()
{
  double supply_voltage = 0.0;
  int num_measurements = 0;

  if (left_motor_)
  {
    const boost::optional<double> motor_supply_voltage = left_motor_->getSupplyVoltage();
    if (motor_supply_voltage)
    {
      supply_voltage += *motor_supply_voltage;
      ++num_measurements;
    }
  }

  if (right_motor_)
  {
    const boost::optional<double> motor_supply_voltage = right_motor_->getSupplyVoltage();
    if (motor_supply_voltage)
    {
      supply_voltage += *motor_supply_voltage;
      ++num_measurements;
    }
  }

  if (steering_motor_)
  {
    const boost::optional<double> motor_supply_voltage = steering_motor_->getSupplyVoltage();
    if (motor_supply_voltage)
    {
      supply_voltage += *motor_supply_voltage;
      ++num_measurements;
    }
  }

  if (num_measurements != 0)
  {
    return supply_voltage / num_measurements;
  }
  return boost::none;
}
}
