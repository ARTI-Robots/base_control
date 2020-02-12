#include <arti_base_control/base_control.h>
#include <angles/angles.h>
#include <arti_base_control/joint_state.h>
#include <arti_base_control/types.h>
#include <functional>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>

namespace arti_base_control
{
BaseControl::BaseControl(const ros::NodeHandle& private_nh)
  : private_nh_(private_nh), reconfigure_server_(private_nh_),
    plugin_loader_("arti_base_control", "arti_base_control::JointActuatorFactory")
{
  cmd_vel_twist_sub_ = private_nh_.subscribe("cmd_vel", 1, &BaseControl::processVelocityCommand, this);
  cmd_ackermann_sub_ = private_nh_.subscribe("cmd_ackermann", 1, &BaseControl::processAckermannCommand, this);

  reconfigure_server_.setCallback(std::bind(&BaseControl::reconfigure, this, std::placeholders::_1));
}

void BaseControl::reconfigure(BaseControlConfig& config)
{
  config_ = config;

  if (!vehicle_)
  {
    const double control_interval = 1.0 / (config_.odometry_rate * 2.1);

    JointActuatorFactoryPtr joint_actuator_factory;
    try
    {
      joint_actuator_factory = plugin_loader_.createInstance(config_.motor_driver);
    }
    catch (const pluginlib::PluginlibException& ex)
    {
      ROS_FATAL_STREAM(
        "Failed to load the " << config_.motor_driver << " as factory for the motor control: " << ex.what());
      throw;
    }

    joint_actuator_factory->init(private_nh_, control_interval, config_.use_mockup);

    if (config_.publish_motor_states)
    {
      joint_actuator_factory = boost::make_shared<PublishingJointActuatorFactory>(joint_actuator_factory);
    }

    vehicle_.emplace(ros::NodeHandle(private_nh_, "vehicle"), joint_actuator_factory);
  }

  if (!odom_pub_ && config_.publish_odom)
  {
    odom_pub_ = private_nh_.advertise<nav_msgs::Odometry>("/odom", 1);
  }
  else if (odom_pub_ && !config_.publish_odom)
  {
    odom_pub_.shutdown();
  }

  if (!tf_broadcaster_ && config_.publish_tf)
  {
    tf_broadcaster_.emplace();
  }
  else if (tf_broadcaster_ && !config_.publish_tf)
  {
    tf_broadcaster_.reset();
  }

  if (!executed_command_pub_ && config_.publish_executed_command)
  {
    executed_command_pub_ = private_nh_.advertise<ackermann_msgs::AckermannDrive>("cmd_ackermann_executed", 1);
  }
  else if (executed_command_pub_ && !config_.publish_executed_command)
  {
    executed_command_pub_.shutdown();
  }

  if (!supply_voltage_pub_ && config_.publish_supply_voltage)
  {
    supply_voltage_pub_ = private_nh_.advertise<std_msgs::Float32>("supply_voltage", 1);
  }
  else if (supply_voltage_pub_ && !config_.publish_supply_voltage)
  {
    supply_voltage_pub_.shutdown();
  }

  if (!joint_states_pub_ && config_.publish_joint_states)
  {
    joint_states_pub_ = private_nh_.advertise<sensor_msgs::JointState>("/joint_states", 1);
  }
  else if (joint_states_pub_ && !config_.publish_joint_states)
  {
    joint_states_pub_.shutdown();
  }

  if (!calculation_infos_pub_ && config_.publish_calculation_info)
  {
    calculation_infos_pub_ = private_nh_.advertise<arti_base_control_msgs::OdometryCalculationInfo>("calculation_infos",
                                                                                                    1);
  }
  else if (calculation_infos_pub_ && !config_.publish_calculation_info)
  {
    calculation_infos_pub_.shutdown();
  }

  if (!odom_timer_)
  {
    odom_timer_ = private_nh_.createTimer(ros::Duration(1.0 / config_.odometry_rate),
                                          &BaseControl::processOdomTimerEvent, this);
  }
}

void BaseControl::processVelocityCommand(const geometry_msgs::TwistConstPtr& cmd_vel)
{
  if (vehicle_)
  {
    vehicle_->setVelocity(*cmd_vel, ros::Time::now());
  }
}

void BaseControl::processAckermannCommand(const ackermann_msgs::AckermannDriveConstPtr& cmd_ackermann)
{
  if (vehicle_)
  {
    vehicle_->setVelocity(*cmd_ackermann, ros::Time::now());
  }
}

void BaseControl::processOdomTimerEvent(const ros::TimerEvent& event)
{
  if (vehicle_)
  {
    const VehicleState vehicle_state = vehicle_->getState(event.current_real);

    geometry_msgs::Twist velocity;
    vehicle_->getVelocity(vehicle_state, velocity);

    arti_base_control_msgs::OdometryCalculationInfo odometry_calculation_info;
    updateOdometry(event.current_real, velocity, odometry_calculation_info);

    if (config_.publish_odom || config_.publish_tf)
    {
      publishOdometry(velocity);
    }

    if (config_.publish_supply_voltage)
    {
      publishSupplyVoltage();
    }

    if (config_.publish_joint_states)
    {
      JointStates joint_states;
      vehicle_->getJointStates(vehicle_state, joint_states);

      sensor_msgs::JointState joint_states_msg;
      joint_states_msg.header.stamp = event.current_real;
      for (const auto& joint_state : joint_states)
      {
        joint_states_msg.name.emplace_back(joint_state.first);
        joint_states_msg.position.emplace_back(joint_state.second.position);
        joint_states_msg.velocity.emplace_back(joint_state.second.velocity);
      }
      joint_states_pub_.publish(joint_states_msg);
    }

    if (config_.publish_executed_command)
    {
      ackermann_msgs::AckermannDrive executed_command;
      vehicle_->getVelocity(vehicle_state, executed_command);
      executed_command_pub_.publish(executed_command);
    }

    if (config_.publish_calculation_info)
    {
      for (const AxleState& axle_state : vehicle_state.axle_states)
      {
        arti_base_control_msgs::OdometryAxleCalculationInfo axle_info;
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
        odometry_calculation_info.axles.emplace_back();
      }
      calculation_infos_pub_.publish(odometry_calculation_info);
    }
  }
}

void BaseControl::updateOdometry(
  const ros::Time& time, const geometry_msgs::Twist& velocity,
  arti_base_control_msgs::OdometryCalculationInfo& odometry_calculation_info)
{
  if (time >= odom_update_time_)
  {
    odometry_calculation_info.odom_velocity = velocity;

    if (!odom_update_time_.isZero())
    {
      const double time_difference = (time - odom_update_time_).toSec();
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
    ROS_WARN("time difference for odom update is negative, skipping update");
  }
}

void BaseControl::publishOdometry(const geometry_msgs::Twist& velocity)
{
  if (!odom_update_time_.isZero())
  {
    const tf::Pose pose(tf::createQuaternionFromYaw(odom_pose_.theta), tf::Vector3(odom_pose_.x, odom_pose_.y, 0.0));

    if (config_.publish_odom)
    {
      nav_msgs::Odometry odom_msg;
      odom_msg.header.stamp = odom_update_time_;
      odom_msg.header.frame_id = config_.odom_frame;
      odom_msg.child_frame_id = config_.base_frame;

      tf::poseTFToMsg(pose, odom_msg.pose.pose);
      odom_msg.pose.covariance[0 * 6 + 0] = config_.odom_x_y_cov;
      odom_msg.pose.covariance[1 * 6 + 1] = config_.odom_x_y_cov;
      odom_msg.pose.covariance[3 * 6 + 3] = config_.odom_yaw_cov;

      odom_msg.twist.twist = velocity;
      odom_msg.twist.covariance[0 * 6 + 0] = config_.odom_x_vel_cov;
      odom_msg.twist.covariance[1 * 6 + 1] = config_.odom_y_vel_cov;
      odom_msg.twist.covariance[3 * 6 + 3] = config_.odom_omega_cov;

      odom_pub_.publish(odom_msg);
    }

    if (config_.publish_tf)
    {
      const tf::StampedTransform transform(pose, odom_update_time_, config_.odom_frame, config_.base_frame);
      tf_broadcaster_->sendTransform(transform);
    }
  }
}

void BaseControl::publishSupplyVoltage()
{
  const boost::optional<double> supply_voltage = vehicle_->getSupplyVoltage();

  if (supply_voltage)
  {
    std_msgs::Float32 msg;
    msg.data = static_cast<std_msgs::Float32::_data_type>(*supply_voltage);
    supply_voltage_pub_.publish(msg);
  }
}
}
