#ifndef ARTI_BASE_CONTROL_TESTS_STAIGHT_DRIVING_TEST_H
#define ARTI_BASE_CONTROL_TESTS_STAIGHT_DRIVING_TEST_H

#include "rclcpp/rclcpp.hpp"

namespace arti_base_control_tests
{
class StaightDrivingTest
{
public:
  explicit StaightDrivingTest(const rclcpp::Node &nh);

  void run();

private:
  double rampTo(double current_command, double target_command, double increment);
  void executeCommandFor(double command, double duration);
  void executeCommand(double command);

  rclcpp::Node nh_;

  ros::Publisher command_publisher_;

  rclcpp::Duration publishing_duration_;

  double target_velocity_;
  double time_to_hold_velocity_;
  double acceleration_time_;
  double stop_time_;
};
}

#endif //ARTI_BASE_CONTROL_TESTS_STAIGHT_DRIVING_TEST_H
