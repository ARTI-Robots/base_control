#include <arti_base_control/base_control.h>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  auto private_nh = std::make_shared<rclcpp::Node>("base_control");

  // const rclcpp::Node private_nh("~");
  arti_base_control::BaseControl base_control(private_nh);

  rclcpp::spin(private_nh);
  rclcpp::shutdown();
  return 0;
}
