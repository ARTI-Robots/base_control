#ifndef ARTI_BASE_CONTROL_TESTS_STRAIGHT_DRIVING_TEST_H
#define ARTI_BASE_CONTROL_TESTS_STRAIGHT_DRIVING_TEST_H

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>

namespace arti_base_control_tests
{
class StraightDrivingTest
{
public:
  explicit StraightDrivingTest(const rclcpp::Node::SharedPtr &nh);

  void run();

private:
  double rampTo(double current_command, double target_command, double increment);
  void executeCommandFor(double command, double duration);
  void executeCommand(double command);

  rclcpp::Node::SharedPtr nh_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
  
  std::chrono::duration<double> publishing_duration_;

  double target_velocity_;
  double time_to_hold_velocity_;
  double acceleration_time_;
  double stop_time_;
  double control_rate_;
};
}

#endif //ARTI_BASE_CONTROL_TESTS_STRAIGHT_DRIVING_TEST_H
