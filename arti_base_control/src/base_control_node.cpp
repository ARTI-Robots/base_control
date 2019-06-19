#include <arti_base_control/base_control.h>
#include <ros/console.h>
#include <ros/init.h>
#include <ros/node_handle.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "base_control");

  const ros::NodeHandle private_nh("~");
  arti_base_control::BaseControl base_control(private_nh);

  ros::spin();

  return 0;
}
