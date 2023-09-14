//
// Created by abuchegger on 08.07.18.
//
#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <arti_base_control/steering.h>
#include <arti_base_control/wheel.h>
#include <arti_base_control/joint_state.h>

TEST(WheelTest, SimpleFrontWheelTest)
{
  const ros::NodeHandle steering_nh("steering");
  auto ideal_ackermann_steering = std::make_shared<arti_base_control::IdealAckermannSteering>(steering_nh);

  const arti_base_control::Wheel wheel(1.0, 0.0, 0.0, 0.5);

  arti_base_control::JointState expected_steering_state;
  expected_steering_state.position = 0.0;
  expected_steering_state.velocity = 0.0;
  const arti_base_control::JointState wheel_steering_state = ideal_ackermann_steering->computeWheelSteeringState(wheel, expected_steering_state);

  EXPECT_DOUBLE_EQ(2.0, wheel.computeWheelVelocity(1.0, 0.0, wheel_steering_state));
  EXPECT_DOUBLE_EQ(2.0 * std::sqrt(2.0), wheel.computeWheelVelocity(1.0, 1.0, wheel_steering_state));
  EXPECT_DOUBLE_EQ(2.0 * std::sqrt(2.0), wheel.computeWheelVelocity(1.0, -1.0, wheel_steering_state));
  EXPECT_DOUBLE_EQ(-2.0 * std::sqrt(2.0), wheel.computeWheelVelocity(-1.0, 1.0, wheel_steering_state));
  EXPECT_DOUBLE_EQ(-2.0 * std::sqrt(2.0), wheel.computeWheelVelocity(-1.0, -1.0, wheel_steering_state));
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
   ros::init(argc, argv, "steering_tester");
  return RUN_ALL_TESTS();
}
