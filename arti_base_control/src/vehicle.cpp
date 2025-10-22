#include <arti_base_control/vehicle.h>
#include <arti_base_control/axle.h>
#include <arti_base_control/utils.h>
#include <boost/range/irange.hpp>
#include <Eigen/Core>
#include <Eigen/QR>
#include <set>
#include <cmath>
#include <functional>

namespace arti_base_control
{

VehicleVelocityConstraint::VehicleVelocityConstraint(double a_v_x_, double a_v_y_, double a_v_theta_, double b_)
  : a_v_x(a_v_x_), a_v_y(a_v_y_), a_v_theta(a_v_theta_), b(b_)
{
}

Vehicle::Vehicle(const rclcpp::Node::SharedPtr& nh,
                 const JointActuatorFactoryPtr& motor_factory,
                 bool process_ackermann)
  : nh_(nh), motor_factory_(motor_factory), process_ackermann_(process_ackermann)
{
  // Load initial parameters (defaults and YAML)
  loadParameters(nh_);

  // Simulate dynamic_reconfigure behavior: update on parameter changes and call reconfigure()
  params_cb_ = nh_->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter>& params)
    {
      for (const auto& p : params)
      {
        const auto& n = p.get_name();
        if (n == "max_velocity_linear")        config_.max_velocity_linear = p.as_double();
        else if (n == "max_velocity_angular")  config_.max_velocity_angular = p.as_double();
        else if (n == "max_steering_angle")    config_.max_steering_angle = p.as_double();
        else if (n == "wheelbase")             config_.wheelbase = p.as_double();
        else if (n == "icr_x")                 config_.icr_x = p.as_double();
        else if (n == "allowed_brake_velocity")config_.allowed_brake_velocity = p.as_double();
        else if (n == "brake_velocity")        config_.brake_velocity = p.as_double();
        else if (n == "brake_current")         config_.brake_current = p.as_double();
      }

      this->reconfigure(config_);

      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      return result;
    });

  // Run reconfigure once to build axles and compute wheelbase as needed
  reconfigure(config_);
}

void Vehicle::loadParameters(const rclcpp::Node::SharedPtr& nh)
{
  // Declare with defaults (so YAML can override them)
  nh->declare_parameter("max_velocity_linear",        config_.max_velocity_linear);
  nh->declare_parameter("max_velocity_angular",       config_.max_velocity_angular);
  nh->declare_parameter("max_steering_angle",         config_.max_steering_angle);
  nh->declare_parameter("wheelbase",                  config_.wheelbase);
  nh->declare_parameter("icr_x",                      config_.icr_x);
  nh->declare_parameter("allowed_brake_velocity",     config_.allowed_brake_velocity);
  nh->declare_parameter("brake_velocity",             config_.brake_velocity);
  nh->declare_parameter("brake_current",              config_.brake_current);

  // Read the current values (after declare, YAML or overrides may be present)
  nh->get_parameter("max_velocity_linear",        config_.max_velocity_linear);
  nh->get_parameter("max_velocity_angular",       config_.max_velocity_angular);
  nh->get_parameter("max_steering_angle",         config_.max_steering_angle);
  nh->get_parameter("wheelbase",                  config_.wheelbase);
  nh->get_parameter("icr_x",                      config_.icr_x);
  nh->get_parameter("allowed_brake_velocity",     config_.allowed_brake_velocity);
  nh->get_parameter("brake_velocity",             config_.brake_velocity);
  nh->get_parameter("brake_current",              config_.brake_current);
}

void Vehicle::reconfigure(VehicleConfig& config)
{
  // Store config
  config_ = config;

  // Keep original diagnostics
  if (config_.max_velocity_linear == 0.0)
  {
    RCLCPP_ERROR(nh_->get_logger(), "Parameter max_velocity_linear is not set");
  }
  if (config_.allowed_brake_velocity == 0.0)
  {
    RCLCPP_WARN(nh_->get_logger(), "Parameter allowed_brake_velocity is not set");
  }
  if (config_.brake_velocity == 0.0)
  {
    RCLCPP_WARN(nh_->get_logger(), "Parameter brake_velocity is not set");
  }
  if (config_.brake_current == 0.0)
  {
    RCLCPP_WARN(nh_->get_logger(), "Parameter brake_current is not set");
  }

  // Build axles from parameters (ROS2 replacement for XmlRpc map)
  if (axles_.empty())
  {
    // Get all parameters under "axles."
    std::vector<rclcpp::Parameter> all_axle_params;
    for (const auto & name : nh_->list_parameters({"axles"}, 10).names) {
      all_axle_params.push_back(nh_->get_parameter(name));
    }

    if (all_axle_params.empty())
    {
      RCLCPP_ERROR(nh_->get_logger(), "axles parameter is missing");
    }
    else
    {
      // Extract unique axle names: keys look like "front_axle.position_x"
      std::set<std::string> axle_names;
      for (const auto& kv : all_axle_params)
      {
        const std::string& full = kv.get_name();               // e.g. "front_axle.position_x"
        const auto dot_pos = full.find('.');
        const std::string axle_name = (dot_pos == std::string::npos) ? full : full.substr(0, dot_pos);
        axle_names.insert(axle_name);
      }

      // Create one node per axle namespace and construct Axle
      for (const auto& axle_name : axle_names)
      {
        // Create a child-like node name "axles/<name>" to keep parameter scoping clear.
        // Axle is expected to fetch its params either by namespace or by prefix "axles.<name>.*".
        const std::string node_name = std::string("axles/") + axle_name;
        auto axle_node = std::make_shared<rclcpp::Node>(node_name, nh_->get_node_options());

        axles_.emplace_back(std::make_shared<Axle>(axle_node, config_, motor_factory_));
      }
    }
  }

  // Compute wheelbase if needed (same logic as ROS1)
  wheelbase_ = config_.wheelbase;
  if (process_ackermann_)
  {
    if (wheelbase_ == 0.0)
    {
      for (const AxlePtr& axle : axles_)
      {
        const AxleConfig& axle_config = axle->getConfig();

        if (axle_config.is_steered)
        {
          wheelbase_ = std::fabs(axle_config.position_x - config_.icr_x);
          if (wheelbase_ != 0.0)
          {
            break;
          }
        }
      }

      if (wheelbase_ == 0.0)
      {
        RCLCPP_WARN(nh_->get_logger(),
          "Wheelbase is not set and could not be determined automatically, this prevents control via Ackermann messages");
      }
    }
  }

  // Propagate vehicle config to axles (same as ROS1)
  for (const AxlePtr& axle : axles_)
  {
    axle->setVehicleConfig(config_);
  }
}

void Vehicle::setVelocity(const ackermann_msgs::msg::AckermannDrive& velocity, const rclcpp::Time& time)
{
  if (!process_ackermann_)
  {
    RCLCPP_ERROR(nh_->get_logger(), "got ackerman command but should not process ackerman commands");
    return;
  }

  const double steering_angle = limit(
    normalizeSteeringAngle(velocity.steering_angle), config_.max_steering_angle, "steering angle");
  const double sin_steering_angle = std::sin(steering_angle);
  const double cos_steering_angle = std::cos(steering_angle);

  // Limit linear velocity to stay below angular velocity limit
  double angular_velocity = 0.0;
  double linear_velocity = limit(velocity.speed, config_.max_velocity_linear, "linear velocity");
  if (linear_velocity != 0.0 && wheelbase_ != 0.0)
  {
    const double a = std::fabs(linear_velocity * sin_steering_angle);
    const double b = std::fabs(config_.max_velocity_angular * wheelbase_ * cos_steering_angle);
    if (a > b)
    {
      const double limited_linear_velocity = linear_velocity * b / a;
      RCLCPP_WARN(rclcpp::get_logger("limit"),
                  "linear velocity (%f) exceeded maximum (%f) due to angular velocity constraint and was limited",
                  linear_velocity, limited_linear_velocity);
      linear_velocity = limited_linear_velocity;
      angular_velocity = config_.max_velocity_angular * (linear_velocity < 0.0 ? -1.0 : 1.0)
                         * (steering_angle < 0.0 ? -1.0 : 1.0);
    }
    else
    {
      angular_velocity = linear_velocity * sin_steering_angle / (wheelbase_ * cos_steering_angle);
    }
  }

  for (const AxlePtr& axle : axles_)
  {
    const AxleConfig& axle_config = axle->getConfig();

    double axle_steering_angle = 0.0;
    if (axle_config.is_steered)
    {
      axle_steering_angle = normalizeSteeringAngle(
        std::atan2(sin_steering_angle * (axle_config.position_x - config_.icr_x),
                   cos_steering_angle * wheelbase_));
    }

    axle->setVelocity(linear_velocity, angular_velocity, axle_steering_angle, time);
  }
}

void Vehicle::setVelocity(const geometry_msgs::msg::Twist& velocity, const rclcpp::Time& time)
{
  const double linear_velocity = limit(velocity.linear.x,  config_.max_velocity_linear,  "linear velocity");
  double angular_velocity      = limit(velocity.angular.z, config_.max_velocity_angular, "angular velocity");

  if (angular_velocity != 0)
  {
    const double a = std::fabs(std::cos(config_.max_steering_angle) * wheelbase_ * angular_velocity);
    const double b = std::fabs(std::sin(config_.max_steering_angle) * linear_velocity);
    if (a > b)
    {
      const double limited_angular_velocity = angular_velocity * b / a;
      RCLCPP_WARN(rclcpp::get_logger("limit"),
                  "angular velocity (%f) exceeded maximum (%f) due to steering angle constraint and was limited",
                  angular_velocity, limited_angular_velocity);
      angular_velocity = limited_angular_velocity;
    }
  }

  for (const AxlePtr& axle : axles_)
  {
    const AxleConfig& axle_config = axle->getConfig();

    double axle_steering_angle = 0.0;
    if (axle_config.is_steered)
    {
      axle_steering_angle = normalizeSteeringAngle(
        std::atan2((axle_config.position_x - config_.icr_x) * angular_velocity, linear_velocity));
    }

    axle->setVelocity(linear_velocity, angular_velocity, axle_steering_angle, time);
  }
}

VehicleState Vehicle::getState(const rclcpp::Time& time) const
{
  VehicleState state;
  for (const AxlePtr& axle : axles_)
  {
    state.axle_states.emplace_back(axle->getState(time));
  }
  return state;
}

void Vehicle::getVelocity(const VehicleState& state, geometry_msgs::msg::Twist& velocity) const
{
  if (state.axle_states.size() != axles_.size())
  {
    throw std::runtime_error("number of axle states in vehicle state differs from number of vehicle axles");
  }

  // Compute vehicle velocity using pseudo inverse of velocity constraints:
  VehicleVelocityConstraints constraints;
  for (const size_t i : boost::irange<size_t>(0, axles_.size()))
  {
    axles_.at(i)->getVelocityConstraints(state.axle_states.at(i), constraints);
  }

  Eigen::MatrixXd a(Eigen::MatrixXd::Zero(constraints.size(), 3));
  Eigen::VectorXd b(Eigen::VectorXd::Zero(constraints.size()));
  for (size_t i = 0; i < constraints.size(); ++i)
  {
    a(i, 0) = constraints[i].a_v_x;
    a(i, 1) = constraints[i].a_v_y;
    a(i, 2) = constraints[i].a_v_theta;
    b(i) = constraints[i].b;
  }

  const Eigen::Vector3d x = a.fullPivHouseholderQr().solve(b);

  velocity.linear.x  = x(0);
  velocity.linear.y  = x(1);
  velocity.linear.z  = 0.0;
  velocity.angular.x = 0.0;
  velocity.angular.y = 0.0;
  velocity.angular.z = x(2);
}

void Vehicle::getVelocity(const VehicleState& state, ackermann_msgs::msg::AckermannDrive& velocity) const
{
  if (state.axle_states.size() != axles_.size())
  {
    throw std::runtime_error("number of axle states in vehicle state differs from number of vehicle axles");
  }

  geometry_msgs::msg::Twist twist;
  getVelocity(state, twist);
  velocity.speed = twist.linear.x;

  // Compute steering angle as average of axle's steering angles:
  double accumulated_steering_angle = 0.0;
  size_t steered_axles_count = 0;

  for (const size_t i : boost::irange<size_t>(0, axles_.size()))
  {
    const AxleState& axle_state = state.axle_states.at(i);
    if (axle_state.steering_motor_state)
    {
      const double axle_steering_angle = axle_state.steering_motor_state->position;
      const AxleConfig& axle_config = axles_.at(i)->getConfig();

      accumulated_steering_angle += normalizeSteeringAngle(std::atan2(
        std::sin(axle_steering_angle) * wheelbase_,
        std::cos(axle_steering_angle) * (axle_config.position_x - config_.icr_x)));
      ++steered_axles_count;
    }
  }

  if (steered_axles_count != 0)
  {
    velocity.steering_angle = accumulated_steering_angle / static_cast<double>(steered_axles_count);
  }
  else
  {
    velocity.steering_angle = 0.0;
  }
}

void Vehicle::getJointStates(const VehicleState& state, JointStates& joint_states) const
{
  if (state.axle_states.size() != axles_.size())
  {
    throw std::runtime_error("number of axle states in vehicle state differs from number of vehicle axles");
  }

  for (const size_t i : boost::irange<size_t>(0, axles_.size()))
  {
    axles_.at(i)->getJointStates(state.axle_states.at(i), joint_states);
  }
}

boost::optional<double> Vehicle::getSupplyVoltage()
{
  double supply_voltage = 0.0;
  int num_measurements = 0;

  for (const AxlePtr& axle : axles_)
  {
    const boost::optional<double> axle_supply_voltage = axle->getSupplyVoltage();
    if (axle_supply_voltage)
    {
      supply_voltage += *axle_supply_voltage;
      ++num_measurements;
    }
  }

  if (num_measurements != 0)
  {
    return supply_voltage / num_measurements;
  }

  return boost::none;
}

double Vehicle::limit(const double value, const double max, const char* name)
{
  const double limited_value = std::min(std::max(-max, value), max);
  if (limited_value != value)
  {
    RCLCPP_WARN(rclcpp::get_logger("limit"), "%s (%f) exceeded maximum (%f) and was limited", name, value, max);
  }
  return limited_value;
}

}  // namespace arti_base_control
