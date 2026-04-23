//
// Created by abuchegger on 08.07.18.
//
#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <arti_base_control/steering.h>
#include <arti_base_control/wheel.h>
#include <arti_base_control/joint_state.h>

class RosTestEnvironment : public ::testing::Environment
{
public:
  RosTestEnvironment(int argc, char ** argv)
  : argc_(argc), argv_(argv)
  {
  }

  void SetUp() override
  {
    rclcpp::init(argc_, argv_);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }

  const int argc_;
  char const * const * argv_;
};

// Global pointer for easy access in tests
RosTestEnvironment * g_ros_env = nullptr;

TEST(WheelTest, SimpleFrontWheelTest)
{
  auto steering_nh = rclcpp::Node::make_shared("steering");
  auto ideal_ackermann_steering = std::make_shared<arti_base_control::IdealAckermannSteering>(steering_nh);

  const arti_base_control::Wheel wheel(1.0, 0.0, 0.0, 0.5);

  arti_base_control::JointState expected_steering_state;
  expected_steering_state.position = 0.0;
  expected_steering_state.velocity = 0.0;
  const arti_base_control::JointState wheel_steering_state = 
    ideal_ackermann_steering->computeWheelSteeringState(wheel, expected_steering_state);

  EXPECT_DOUBLE_EQ(2.0, wheel.computeWheelVelocity(1.0, 0.0, wheel_steering_state));
  EXPECT_DOUBLE_EQ(2.0 * std::sqrt(2.0), wheel.computeWheelVelocity(1.0, 1.0, wheel_steering_state));
  EXPECT_DOUBLE_EQ(2.0 * std::sqrt(2.0), wheel.computeWheelVelocity(1.0, -1.0, wheel_steering_state));
  EXPECT_DOUBLE_EQ(-2.0 * std::sqrt(2.0), wheel.computeWheelVelocity(-1.0, 1.0, wheel_steering_state));
  EXPECT_DOUBLE_EQ(-2.0 * std::sqrt(2.0), wheel.computeWheelVelocity(-1.0, -1.0, wheel_steering_state));
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  g_ros_env = static_cast<RosTestEnvironment *>(
    ::testing::AddGlobalTestEnvironment(new RosTestEnvironment(argc, argv))
  );
  return RUN_ALL_TESTS();
}
