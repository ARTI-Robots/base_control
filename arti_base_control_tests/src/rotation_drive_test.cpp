#include <arti_base_control_tests/rotation_drive_test.h>
#include <geometry_msgs/Twist.h>

namespace arti_base_control_tests
{
RotationDriveTest::RotationDriveTest(const ros::NodeHandle& nh) : nh_(nh)
{
  command_publisher_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  const double control_rate = nh_.param<double>("control_rate", 20.);
  publishing_duration_ = ros::Duration(1. / control_rate);

  target_velocity_ = nh_.param<double>("target_velocity", 0.3);
  time_to_hold_velocity_ = nh_.param<double>("time_to_hold_velocity", 1.);
  acceleration_time_ = nh_.param<double>("acceleration_time", 1.);
  stop_time_ = nh_.param<double>("stop_time", 0.2);
}

void RotationDriveTest::run()
{
  // calculate acceleration steps
  const double acceleration_steps = target_velocity_ / (acceleration_time_ / publishing_duration_.toSec());

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

double RotationDriveTest::rampTo(double current_command, double target_command, double increment)
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
    publishing_duration_.sleep();
  }

  return current_command;
}

void RotationDriveTest::executeCommandFor(double command, double duration)
{
  ros::Time end_time = ros::Time::now() + ros::Duration(duration);

  while (ros::Time::now() < end_time)
  {
    executeCommand(command);
    publishing_duration_.sleep();
  }
}

void RotationDriveTest::executeCommand(double command)
{
  geometry_msgs::Twist command_msg;
  command_msg.angular.z = command;
  command_publisher_.publish(command_msg);
  ros::spinOnce();
}

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rotation_drive_test");

  ros::NodeHandle nh("~");
  arti_base_control_tests::RotationDriveTest node(nh);
  node.run();
  return 0;
}
