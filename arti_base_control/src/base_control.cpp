#include <arti_base_control/base_control.h>

#include <angles/angles.h>
#include <arti_base_control/joint_state.h>
#include <arti_base_control/types.h>

#include <functional>
#include <chrono>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32.hpp>

namespace arti_base_control
{
BaseControl::BaseControl(const rclcpp::Node::SharedPtr& private_nh)
  : private_nh_(private_nh), 
    plugin_loader_("arti_base_control", "arti_base_control::JointActuatorFactory")
{

  private_nh_->declare_parameter<bool>("publish_odom",               config_.publish_odom);
  private_nh_->declare_parameter<bool>("publish_tf",                 config_.publish_tf);
  private_nh_->declare_parameter<bool>("publish_executed_command",   config_.publish_executed_command);
  private_nh_->declare_parameter<bool>("publish_motor_states",       config_.publish_motor_states);
  private_nh_->declare_parameter<bool>("publish_joint_states",       config_.publish_joint_states);
  private_nh_->declare_parameter<bool>("publish_supply_voltage",     config_.publish_supply_voltage);
  private_nh_->declare_parameter<bool>("publish_calculation_info",   config_.publish_calculation_info);
  private_nh_->declare_parameter<bool>("execute_ackermann_commands", config_.execute_ackermann_commands);

  private_nh_->declare_parameter<std::string>("odom_frame",          config_.odom_frame);
  private_nh_->declare_parameter<std::string>("base_frame",          config_.base_frame);

  private_nh_->declare_parameter<double>("odom_x_y_cov",             config_.odom_x_y_cov);
  private_nh_->declare_parameter<double>("odom_yaw_cov",             config_.odom_yaw_cov);
  private_nh_->declare_parameter<double>("odom_x_vel_cov",           config_.odom_x_vel_cov);
  private_nh_->declare_parameter<double>("odom_y_vel_cov",           config_.odom_y_vel_cov);
  private_nh_->declare_parameter<double>("odom_omega_cov",           config_.odom_omega_cov);

  private_nh_->declare_parameter<std::string>("motor_driver",        config_.motor_driver);

  config_.publish_odom               = private_nh_->get_parameter("publish_odom").as_bool();
  config_.publish_tf                 = private_nh_->get_parameter("publish_tf").as_bool();
  config_.publish_executed_command   = private_nh_->get_parameter("publish_executed_command").as_bool();
  config_.publish_motor_states       = private_nh_->get_parameter("publish_motor_states").as_bool();
  config_.publish_joint_states       = private_nh_->get_parameter("publish_joint_states").as_bool();
  config_.publish_supply_voltage     = private_nh_->get_parameter("publish_supply_voltage").as_bool();
  config_.publish_calculation_info   = private_nh_->get_parameter("publish_calculation_info").as_bool();
  config_.execute_ackermann_commands = private_nh_->get_parameter("execute_ackermann_commands").as_bool();

  config_.odom_frame                 = private_nh_->get_parameter("odom_frame").as_string();
  config_.base_frame                 = private_nh_->get_parameter("base_frame").as_string();
  config_.odometry_rate              = private_nh_->get_parameter("odometry_rate").as_double();
  config_.use_mockup                 = private_nh_->get_parameter("use_mockup").as_bool();

  config_.odom_x_y_cov               = private_nh_->get_parameter("odom_x_y_cov").as_double();
  config_.odom_yaw_cov               = private_nh_->get_parameter("odom_yaw_cov").as_double();
  config_.odom_x_vel_cov             = private_nh_->get_parameter("odom_x_vel_cov").as_double();
  config_.odom_y_vel_cov             = private_nh_->get_parameter("odom_y_vel_cov").as_double();
  config_.odom_omega_cov             = private_nh_->get_parameter("odom_omega_cov").as_double();

  config_.motor_driver               = private_nh_->get_parameter("motor_driver").as_string();
  // cmd_vel_twist_sub_ = private_nh_.subscribe("cmd_vel", 1, &BaseControl::processVelocityCommand, this);
  params_cb_handle_ = private_nh_->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter>& params)
    {
      BaseControlConfig new_cfg = this->config_;

      for (const auto& p : params)
      {
        const auto& name = p.get_name();
        if      (name == "publish_odom") new_cfg.publish_odom = p.as_bool();
        else if (name == "publish_tf") new_cfg.publish_tf = p.as_bool();
        else if (name == "publish_executed_command") new_cfg.publish_executed_command = p.as_bool();
        else if (name == "publish_motor_states") new_cfg.publish_motor_states = p.as_bool();
        else if (name == "publish_joint_states") new_cfg.publish_joint_states = p.as_bool();
        else if (name == "publish_supply_voltage") new_cfg.publish_supply_voltage = p.as_bool();
        else if (name == "publish_calculation_info") new_cfg.publish_calculation_info = p.as_bool();
        else if (name == "execute_ackermann_commands") new_cfg.execute_ackermann_commands = p.as_bool();
        else if (name == "odom_frame") new_cfg.odom_frame = p.as_string();
        else if (name == "base_frame") new_cfg.base_frame = p.as_string();
        else if (name == "odometry_rate") new_cfg.odometry_rate = p.as_double();
        else if (name == "use_mockup") new_cfg.use_mockup = p.as_bool();
        else if (name == "odom_x_y_cov") new_cfg.odom_x_y_cov = p.as_double();
        else if (name == "odom_yaw_cov") new_cfg.odom_yaw_cov = p.as_double();
        else if (name == "odom_x_vel_cov") new_cfg.odom_x_vel_cov = p.as_double();
        else if (name == "odom_y_vel_cov") new_cfg.odom_y_vel_cov = p.as_double();
        else if (name == "odom_omega_cov") new_cfg.odom_omega_cov = p.as_double();
        else if (name == "motor_driver") new_cfg.motor_driver = p.as_string();
      }

      if (new_cfg.odometry_rate <= 0.0)
      {
        rcl_interfaces::msg::SetParametersResult r;
        r.successful = false;
        r.reason = "odometry_rate must be > 0.0";
        return r;
      }

      this->reconfigure(new_cfg);

      rcl_interfaces::msg::SetParametersResult r;
      r.successful = true;
      return r;
    }
  );

  cmd_vel_twist_sub_ = private_nh_->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::QoS(10),
    std::bind(&BaseControl::processVelocityCommand, this, std::placeholders::_1));
  
  this->reconfigure(config_); // ROS1-like: simulate the first dynamic_reconfigure call
  RCLCPP_INFO(private_nh_->get_logger(), "BaseControl initialized (ROS 2, ROS1-style reconfigure).");
}

void BaseControl::reconfigure(const BaseControlConfig& config)
{
  BaseControlConfig old = config_;
  config_ = config;

  if (!vehicle_)
  {
    const double control_interval = 1.0 / (config_.odometry_rate * 2.1);

    try
    {
      JointActuatorFactoryPtr factory = plugin_loader_.createSharedInstance(config_.motor_driver);
      factory->init(private_nh_, control_interval, config_.use_mockup);

      if (config_.publish_motor_states)
        factory = std::make_shared<PublishingJointActuatorFactory>(factory);

      auto vehicle_node = rclcpp::Node::make_shared(private_nh_->get_name() + std::string("_vehicle"));
      vehicle_.emplace(vehicle_node, factory, config_.execute_ackermann_commands);
    }
    catch (const pluginlib::PluginlibException& ex)
    {
      RCLCPP_FATAL(private_nh_->get_logger(),
                   "Failed to load motor driver '%s': %s",
                   config_.motor_driver.c_str(), ex.what());
      throw;
    }
  }

  if (!odom_pub_ && config_.publish_odom)
  {
    odom_pub_ = private_nh_->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(10));
  }
  else if (odom_pub_ && !config_.publish_odom)
  {
    odom_pub_.reset();
  }

  if (!tf_broadcaster_ && config_.publish_tf)
  {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(private_nh_);
  }
  else if (tf_broadcaster_ && !config_.publish_tf)
  {
    tf_broadcaster_.reset();
  }

  if (!executed_command_pub_ && config_.publish_executed_command && config_.execute_ackermann_commands)
  {
   executed_command_pub_ =
      private_nh_->create_publisher<ackermann_msgs::msg::AckermannDrive>("cmd_ackermann_executed", rclcpp::QoS(10));
  }
  else if (executed_command_pub_ && (!config_.publish_executed_command || !config_.execute_ackermann_commands))
  {
    executed_command_pub_.reset();
  }

  if (!cmd_ackermann_sub_ && config_.execute_ackermann_commands)
  {
    cmd_ackermann_sub_ = private_nh_->create_subscription<ackermann_msgs::msg::AckermannDrive>(
      "cmd_ackermann", rclcpp::QoS(10),
      std::bind(&BaseControl::processAckermannCommand, this, std::placeholders::_1));
  }
  else if (cmd_ackermann_sub_ && !config_.execute_ackermann_commands)
  {
    cmd_ackermann_sub_.reset();
  }

  if (!supply_voltage_pub_ && config_.publish_supply_voltage)
  {
    supply_voltage_pub_ = private_nh_->create_publisher<std_msgs::msg::Float32>("supply_voltage", rclcpp::QoS(10));
  }
  else if (supply_voltage_pub_ && !config_.publish_supply_voltage)
  {
    supply_voltage_pub_.reset();
  }

  if (!joint_states_pub_ && config_.publish_joint_states)
  {
    joint_states_pub_ = private_nh_->create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::QoS(10));
  }
  else if (joint_states_pub_ && !config_.publish_joint_states)
  {
    joint_states_pub_.reset();
  }

  if (!calculation_infos_pub_ && config_.publish_calculation_info)
  {
    calculation_infos_pub_ =
      private_nh_->create_publisher<arti_base_control_msgs::msg::OdometryCalculationInfo>("calculation_infos", rclcpp::QoS(10));
  }
  else if (calculation_infos_pub_ && !config_.publish_calculation_info)
  {
    calculation_infos_pub_.reset();
  }

  if (!odom_timer_)
  {
    using namespace std::chrono;
    const double hz = std::max(1e-6, config_.odometry_rate);
    const auto period = duration_cast<nanoseconds>(duration<double>(1.0 / hz));
    odom_timer_ = private_nh_->create_wall_timer(period, std::bind(&BaseControl::processOdomTimer, this));
  }
}

void BaseControl::processVelocityCommand(const geometry_msgs::msg::Twist::ConstSharedPtr& cmd_vel)
{
  if (vehicle_)
  {
    vehicle_->setVelocity(*cmd_vel, private_nh_->now());
  }
}

void BaseControl::processAckermannCommand(const ackermann_msgs::msg::AckermannDrive::ConstSharedPtr& cmd_ackermann)
{
  if (vehicle_)
  {
    vehicle_->setVelocity(*cmd_ackermann, private_nh_->now());
  }
}

void BaseControl::processOdomTimer()
{
  if (vehicle_)
  {
    const rclcpp::Time now = private_nh_->now();
    const VehicleState vehicle_state = vehicle_->getState(now);

    geometry_msgs::msg::Twist velocity;
    vehicle_->getVelocity(vehicle_state, velocity);

    arti_base_control_msgs::msg::OdometryCalculationInfo odometry_calculation_info;
    const rclcpp::Time now_time = private_nh_->now();
    updateOdometry(now_time, velocity, odometry_calculation_info);

    if (config_.publish_odom || config_.publish_tf)
    {
      publishOdometry(velocity);
    }

    if (config_.publish_supply_voltage)
    {
      publishSupplyVoltage();
    }

    if (config_.publish_joint_states && joint_states_pub_)
    {
      JointStates joint_states;
      vehicle_->getJointStates(vehicle_state, joint_states);

      sensor_msgs::msg::JointState joint_states_msg;
      joint_states_msg.header.stamp = now;
      for (const auto& joint_state : joint_states)
      {
        joint_states_msg.name.emplace_back(joint_state.first);
        joint_states_msg.position.emplace_back(joint_state.second.position);
        joint_states_msg.velocity.emplace_back(joint_state.second.velocity);
      }
      joint_states_pub_->publish(joint_states_msg);
    }

    if (config_.publish_executed_command && config_.execute_ackermann_commands)
    {
      ackermann_msgs::msg::AckermannDrive executed_command;
      vehicle_->getVelocity(vehicle_state, executed_command);
      executed_command_pub_->publish(executed_command);
    }

    if (config_.publish_calculation_info)
    {
      for (const AxleState& axle_state : vehicle_state.axle_states)
      {
        arti_base_control_msgs::msg::OdometryAxleCalculationInfo axle_info;
        if (axle_state.steering_motor_state)
        {
          axle_info.steering_angle = axle_state.steering_motor_state->position;
          axle_info.steering_velocity = axle_state.steering_motor_state->velocity;
        }

        if (axle_state.left_motor_state)
        {
          axle_info.left_velocity = axle_state.left_motor_state->velocity;
        }

        if (axle_state.right_motor_state)
        {
          axle_info.right_velocity = axle_state.right_motor_state->velocity;
        }
        odometry_calculation_info.axles.emplace_back(axle_info);
      }
      calculation_infos_pub_->publish(odometry_calculation_info);
    }
  }
}

void BaseControl::updateOdometry(
  const rclcpp::Time& time, 
  const geometry_msgs::msg::Twist& velocity,
  arti_base_control_msgs::msg::OdometryCalculationInfo& odometry_calculation_info)
{
  if (time >= odom_update_time_)
  {
    odometry_calculation_info.odom_velocity = velocity;

    if (odom_update_time_.nanoseconds() != 0 && time < odom_update_time_)
    {
      const double time_difference = (time - odom_update_time_).seconds();
      odometry_calculation_info.time_difference = time_difference;

      const double sin_yaw = std::sin(odom_pose_.theta);
      const double cos_yaw = std::cos(odom_pose_.theta);

      odom_pose_.x += (velocity.linear.x * cos_yaw - velocity.linear.y * sin_yaw) * time_difference;
      odom_pose_.y += (velocity.linear.x * sin_yaw + velocity.linear.y * cos_yaw) * time_difference;
      odom_pose_.theta = angles::normalize_angle(odom_pose_.theta + velocity.angular.z * time_difference);
    }

    odometry_calculation_info.odom_pose = odom_pose_;
    odom_update_time_ = time;
  }
  else
  {
    RCLCPP_WARN(private_nh_->get_logger(), "Negative time delta, skipping odometry update.");
  }
}

void BaseControl::publishOdometry(const geometry_msgs::msg::Twist& velocity)
{
  if (!(odom_update_time_.nanoseconds() == 0))
  {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, odom_pose_.theta);
    geometry_msgs::msg::Quaternion q_msg = tf2::toMsg(q);

    if (config_.publish_odom)
    {
      nav_msgs::msg::Odometry odom_msg;
      odom_msg.header.stamp = odom_update_time_;
      odom_msg.header.frame_id = config_.odom_frame;
      odom_msg.child_frame_id = config_.base_frame;

      odom_msg.pose.pose.position.x = odom_pose_.x;
      odom_msg.pose.pose.position.y = odom_pose_.y;
      odom_msg.pose.pose.orientation = q_msg;

      odom_msg.pose.covariance[0 * 6 + 0] = config_.odom_x_y_cov;
      odom_msg.pose.covariance[1 * 6 + 1] = config_.odom_x_y_cov;
      odom_msg.pose.covariance[5 * 6 + 5] = config_.odom_yaw_cov;

      odom_msg.twist.twist = velocity;
      odom_msg.twist.covariance[0 * 6 + 0] = config_.odom_x_vel_cov;
      odom_msg.twist.covariance[1 * 6 + 1] = config_.odom_y_vel_cov;
      odom_msg.twist.covariance[5 * 6 + 5] = config_.odom_omega_cov;

      odom_pub_->publish(odom_msg);
    }

    if (config_.publish_tf)
    {
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header.stamp = odom_update_time_;
      tf_msg.header.frame_id = config_.odom_frame;
      tf_msg.child_frame_id = config_.base_frame;
      tf_msg.transform.translation.x = odom_pose_.x;
      tf_msg.transform.translation.y = odom_pose_.y;
      tf_msg.transform.translation.z = 0.0;
      tf_msg.transform.rotation = q_msg;
      tf_broadcaster_->sendTransform(tf_msg);
    }
  }
}

void BaseControl::publishSupplyVoltage()
{
  boost::optional<double> supply_voltage = vehicle_->getSupplyVoltage();

  if (config_.use_mockup == true)
  {
    supply_voltage = 40.0;
  }

  if (supply_voltage && supply_voltage_pub_)
  {
    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(*supply_voltage);
    supply_voltage_pub_->publish(msg);
  }
}
}
