#include <arti_base_control/axle.h>
#include <arti_base_control/velocity_controlled_joint_actuator.h>
#include <arti_base_control/joint_actuator_factory.h>
#include <arti_base_control/steering.h>
#include <arti_base_control/position_controlled_joint_actuator.h>
#include <arti_base_control/utils.h>
#include <arti_base_control/vehicle.h>

#include <cmath>
#include <string>

#include <rcl_interfaces/msg/set_parameters_result.hpp>

namespace arti_base_control
{

Axle::Axle(const rclcpp::Node::SharedPtr& nh,
           const VehicleConfig& vehicle_config,
           const JointActuatorFactoryPtr& motor_factory)
  : nh_(nh), motor_factory_(motor_factory), vehicle_config_(vehicle_config)
{

  // Declare parameters (ROS2 version of dynamic_reconfigure initial load) 
  nh_->declare_parameter<double>("axle.wheel_diameter", 0.0);
  nh_->declare_parameter<double>("axle.position_x", 0.0);
  nh_->declare_parameter<double>("axle.position_y", 0.0);
  nh_->declare_parameter<double>("axle.track", 0.0);
  nh_->declare_parameter<double>("axle.steering_hinge_offset", 0.0);
  nh_->declare_parameter<bool>("axle.is_steered", false);
  nh_->declare_parameter<bool>("axle.is_driven", false);
  nh_->declare_parameter<double>("axle.steering_velocity", 0.0);
  nh_->declare_parameter<double>("axle.steering_position_tolerance", 0.001);

  nh_->declare_parameter<std::string>("axle.left_hinge_joint", "");
  nh_->declare_parameter<std::string>("axle.left_wheel_joint", "");
  nh_->declare_parameter<std::string>("axle.right_hinge_joint", "");
  nh_->declare_parameter<std::string>("axle.right_wheel_joint", "");

  // Load configuration from parameters
  AxleConfig cfg{};
  cfg.wheel_diameter = nh_->get_parameter("axle.wheel_diameter").as_double();
  cfg.position_x = nh_->get_parameter("axle.position_x").as_double();
  cfg.position_y = nh_->get_parameter("axle.position_y").as_double();
  cfg.track = nh_->get_parameter("axle.track").as_double();
  cfg.steering_hinge_offset = nh_->get_parameter("axle.steering_hinge_offset").as_double();
  cfg.is_steered = nh_->get_parameter("axle.is_steered").as_bool();
  cfg.is_driven = nh_->get_parameter("axle.is_driven").as_bool();
  cfg.steering_velocity = nh_->get_parameter("axle.steering_velocity").as_double();
  cfg.steering_position_tolerance = nh_->get_parameter("axle.steering_position_tolerance").as_double();

  cfg.left_hinge_joint = nh_->get_parameter("axle.left_hinge_joint").as_string();
  cfg.left_wheel_joint = nh_->get_parameter("axle.left_wheel_joint").as_string();
  cfg.right_hinge_joint = nh_->get_parameter("axle.right_hinge_joint").as_string();
  cfg.right_wheel_joint = nh_->get_parameter("axle.right_wheel_joint").as_string();


  reconfigure(cfg);

  // Dynamic callback (equivalent to dynamic_reconfigure in ROS1)
  params_cb_handle_ = nh_->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter>& params)
    {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      result.reason = "";

      AxleConfig new_cfg = *config_;

      for (const auto& p : params)
      {
        const auto& name = p.get_name();

        if (name == "axle.steering_velocity")
        {
          new_cfg.steering_velocity = p.as_double();
        }
        else if (name == "axle.steering_position_tolerance")
        {
          new_cfg.steering_position_tolerance = p.as_double();
        }
        else if (name == "axle.wheel_diameter")
        {
          new_cfg.wheel_diameter = p.as_double();
        }
        else if (name == "axle.is_driven" || name == "axle.is_steered")
        {
          RCLCPP_WARN(nh_->get_logger(),
                      "Attempted to modify '%s' dynamically; ignoring.",
                      name.c_str());
          // return rclcpp::SetParametersResult{false};
          continue;
        }
      }

      reconfigure(new_cfg);
      RCLCPP_INFO(nh_->get_logger(), "Axle parameters updated dynamically.");

      return result;
    });
}

void Axle::reconfigure(AxleConfig& config)
{
  if (config.wheel_diameter == 0.0)
  {
    RCLCPP_ERROR(nh_->get_logger(), "Parameter wheel_diameter is zero");
  }

  if (config_)
  {
    if (config.is_steered != config_->is_steered)
    {
      RCLCPP_ERROR(nh_->get_logger(), "Parameter is_steered cannot be changed dynamically");
      config.is_steered = config_->is_steered;
    }

    if (config.is_driven != config_->is_driven)
    {
      RCLCPP_ERROR(nh_->get_logger(), "Parameter is_driven cannot be changed dynamically");
      config.is_driven = config_->is_driven;
    }
  }
  else
  {
    if (config.is_steered)
    {
      auto steering_nh = nh_->create_sub_node("steering");
      steering_nh->declare_parameter<std::string>("type", "");
      std::string steering_type = steering_nh->get_parameter("type").as_string();

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
        RCLCPP_ERROR(nh_->get_logger(), "Unknown steering type '%s'", steering_type.c_str());
      }

      auto steering_motor_nh = nh_->create_sub_node("steering_motor");
      steering_motor_ = motor_factory_->createPositionControlledJointActuator(steering_motor_nh);
    }

    if (config.is_driven)
    {
      auto left_motor_nh = nh_->create_sub_node("left_motor");
      left_motor_ = motor_factory_->createVelocityControlledJointActuator(left_motor_nh);

      auto right_motor_nh = nh_->create_sub_node("right_motor");
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
  if (!config_)
  {
    throw std::logic_error("Axle configuration accessed before initialization");
  }
  return *config_;
}

void Axle::setVehicleConfig(const VehicleConfig& vehicle_config)
{
  vehicle_config_ = vehicle_config;
}

void Axle::setVelocity(double linear_velocity,
                       double angular_velocity,
                       double axle_steering_angle,
                       const rclcpp::Time& time)
{
  JointState expected_steering_state;

  // --- Steering (if exists) ---
  if (steering_motor_ && steering_)
  {
    const double left_angle =
      left_wheel_.computeIdealWheelSteeringAngle(axle_steering_angle, vehicle_config_.icr_x);
    const double right_angle =
      right_wheel_.computeIdealWheelSteeringAngle(axle_steering_angle, vehicle_config_.icr_x);

    const double left_pos =
      steering_->computeSteeringPosition(left_wheel_, left_angle);
    const double right_pos =
      steering_->computeSteeringPosition(right_wheel_, right_angle);
    const double target_pos = 0.5 * (left_pos + right_pos);

    const JointState current = steering_motor_->getState(time);
    expected_steering_state.position = current.position;

    const double error = target_pos - current.position;
    if (error > config_->steering_position_tolerance)
    {
      expected_steering_state.velocity = config_->steering_velocity;
    }
    else if (error < -config_->steering_position_tolerance)
    {
      expected_steering_state.velocity = -config_->steering_velocity;
    }

    steering_motor_->setPosition(target_pos);
  }

  // --- Drive wheels ---
  if (left_motor_ && right_motor_)
  {
    const JointState left_steer =
      steering_ ? steering_->computeWheelSteeringState(left_wheel_, expected_steering_state) : JointState();
    const JointState right_steer =
      steering_ ? steering_->computeWheelSteeringState(right_wheel_, expected_steering_state) : JointState();

    const double left_vel =
      left_wheel_.computeWheelVelocity(linear_velocity, angular_velocity, left_steer);
    const double right_vel =
      right_wheel_.computeWheelVelocity(linear_velocity, angular_velocity, right_steer);

    const bool request_stopped =
      (std::fabs(left_vel) <= vehicle_config_.brake_velocity) &&
      (std::fabs(right_vel) <= vehicle_config_.brake_velocity);
    const bool actual_stopped =
      (std::fabs(left_motor_->getState(time).velocity) <= vehicle_config_.allowed_brake_velocity) &&
      (std::fabs(right_motor_->getState(time).velocity) <= vehicle_config_.allowed_brake_velocity);

    if (request_stopped && actual_stopped)
    {
      left_motor_->brake(vehicle_config_.brake_current);
      right_motor_->brake(vehicle_config_.brake_current);
    }
    else
    {
      left_motor_->setVelocity(left_vel);
      right_motor_->setVelocity(right_vel);
    }
  }
}

AxleState Axle::getState(const rclcpp::Time& time) const
{
  AxleState state;
  if (steering_motor_) state.steering_motor_state.emplace(steering_motor_->getState(time));
  if (left_motor_) state.left_motor_state.emplace(left_motor_->getState(time));
  if (right_motor_) state.right_motor_state.emplace(right_motor_->getState(time));
  return state;
}

void Axle::getVelocityConstraints(const AxleState& state, VehicleVelocityConstraints& constraints) const
{
  const JointState steer = state.steering_motor_state.get_value_or(JointState(0.0, 0.0));

  const JointState left_steer =
    steering_ ? steering_->computeWheelSteeringState(left_wheel_, steer) : JointState();
  const JointState right_steer =
    steering_ ? steering_->computeWheelSteeringState(right_wheel_, steer) : JointState();

  left_wheel_.computeVehicleVelocityConstraints(state.left_motor_state, left_steer, constraints);
  right_wheel_.computeVehicleVelocityConstraints(state.right_motor_state, right_steer, constraints);
}

void Axle::getJointStates(const AxleState& state, JointStates& joint_states) const
{
  if (!config_) return;

  if (state.left_motor_state && !config_->left_wheel_joint.empty())
    joint_states[config_->left_wheel_joint] = *state.left_motor_state;
  if (state.right_motor_state && !config_->right_wheel_joint.empty())
    joint_states[config_->right_wheel_joint] = *state.right_motor_state;

  if (state.steering_motor_state && steering_)
  {
    steering_->getJointStates(*state.steering_motor_state, joint_states);

    if (!config_->left_hinge_joint.empty())
      joint_states[config_->left_hinge_joint] =
        steering_->computeWheelSteeringState(left_wheel_, *state.steering_motor_state);

    if (!config_->right_hinge_joint.empty())
      joint_states[config_->right_hinge_joint] =
        steering_->computeWheelSteeringState(right_wheel_, *state.steering_motor_state);
  }
}

boost::optional<double> Axle::getSupplyVoltage()
{
  double sum = 0.0;
  int n = 0;

  auto add = [&](const auto& motor)
  {
    if (motor)
    {
      const auto v = motor->getSupplyVoltage();
      if (v) { sum += *v; ++n; }
    }
  };

  add(left_motor_);
  add(right_motor_);
  add(steering_motor_);

  if (n > 0)
    return sum / static_cast<double>(n);
  return boost::none;
}

} // namespace arti_base_control
