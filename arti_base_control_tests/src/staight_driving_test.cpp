#include <arti_base_control_tests/staight_driving_test.h>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>

namespace arti_base_control_tests
{
StaightDrivingTest::StaightDrivingTest(const rclcpp::Node::SharedPtr& nh) 
  : nh_(nh),
    publishing_duration_(std::chrono::duration<double>(0.0))
{
  command_publisher_ = nh_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

  // Declare and retrieve parameters (ROS 2 style)
  nh_->declare_parameter<double>("control_rate", 20.0);
  nh_->declare_parameter<double>("target_velocity", 1.0);
  nh_->declare_parameter<double>("time_to_hold_velocity", 1.0);
  nh_->declare_parameter<double>("acceleration_time", 1.0);
  nh_->declare_parameter<double>("stop_time", 0.2);

  // double control_rate_;
  nh_->get_parameter("control_rate", control_rate_);
  nh_->get_parameter("target_velocity", target_velocity_);
  nh_->get_parameter("time_to_hold_velocity", time_to_hold_velocity_);
  nh_->get_parameter("acceleration_time", acceleration_time_);
  nh_->get_parameter("stop_time", stop_time_);

  // Duration between commands (inverse of control rate)
  publishing_duration_ = std::chrono::duration<double>(1.0 / control_rate_);
}

void StaightDrivingTest::run()
{
  // calculate acceleration steps
  const double acceleration_steps = target_velocity_ / (acceleration_time_ / publishing_duration_.count());

  // first ramp up to target velocity
  double real_velocity = rampTo(0., target_velocity_, acceleration_steps);

  // hold target velocity for specified time
  executeCommandFor(real_velocity, time_to_hold_velocity_);

  // ramp down to 0.
  rampTo(real_velocity, 0., acceleration_steps);

  // stand for stop time
  executeCommandFor(0., stop_time_);

  // move back with the same ramp up and down behaviour
  // first ramp up to target velocity
  real_velocity = rampTo(0., -target_velocity_, acceleration_steps);

  // hold target velocity for specified time
  executeCommandFor(real_velocity, time_to_hold_velocity_);

  // ramp down to 0.
  rampTo(real_velocity, 0., acceleration_steps);

  // stand for stop time
  executeCommandFor(0., stop_time_);
}

double StaightDrivingTest::rampTo(double current_command, double target_command, double increment)
{
  bool ramp_up = current_command < target_command;

  if ((!ramp_up && (increment > 0.)) || (ramp_up && (increment < 0.)))
  {
    increment *= -1.;
  }

  while ((ramp_up && (current_command < target_command)) || (!ramp_up && (current_command > target_command)))
  {
    current_command += increment;
    executeCommand(current_command);
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(publishing_duration_));
  }

  return current_command;
}

void StaightDrivingTest::executeCommandFor(double command, double duration)
{
  rclcpp::Time end_time = nh_->get_clock()->now() + rclcpp::Duration::from_seconds(duration);

  while (nh_->get_clock()->now() < end_time)
  {
    executeCommand(command);
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(publishing_duration_));
  }
}

void StaightDrivingTest::executeCommand(double command)
{
  geometry_msgs::msg::Twist command_msg;
  command_msg.linear.x = command;
  command_publisher_->publish(command_msg);
  rclcpp::spin_some(nh_);
}

}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("staight_driving_test");
  auto test = std::make_shared<arti_base_control_tests::StaightDrivingTest>(node);
  test->run();
  rclcpp::shutdown();
  return 0;
}
