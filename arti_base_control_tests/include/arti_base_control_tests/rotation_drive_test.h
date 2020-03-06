#ifndef ARTI_BASE_CONTROL_TESTS_ROTATION_DRIVE_TEST_H
#define ARTI_BASE_CONTROL_TESTS_ROTATION_DRIVE_TEST_H

#include <ros/ros.h>

namespace arti_base_control_tests
{
class RotationDriveTest
{
public:
  explicit RotationDriveTest(const ros::NodeHandle &nh);

  void run();

private:
  double rampTo(double current_command, double target_command, double increment);
  void executeCommandFor(double command, double duration);
  void executeCommand(double command);

  ros::NodeHandle nh_;

  ros::Publisher command_publisher_;

  ros::Duration publishing_duration_;

  double target_velocity_;
  double time_to_hold_velocity_;
  double acceleration_time_;
  double stop_time_;
};
}

#endif //ARTI_BASE_CONTROL_TESTS_ROTATION_DRIVE_TEST_H
