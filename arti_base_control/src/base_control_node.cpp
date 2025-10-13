#include <arti_base_control/base_control.h>
#include <ros/console.h>
#include <ros/init.h>
#include <ros/node_handle.h>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("base_control");

  const rclcpp::Node private_nh("~");
  arti_base_control::BaseControl base_control(private_nh);

  rclcpp::spin(node);

  return 0;
}
