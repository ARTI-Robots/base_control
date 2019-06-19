#ifndef ARTI_BASE_CONTROL_BASE_CONTROL_H
#define ARTI_BASE_CONTROL_BASE_CONTROL_H

#include <ackermann_msgs/AckermannDrive.h>
#include <arti_base_control/BaseControlConfig.h>
#include <arti_base_control/types.h>
#include <arti_base_control_msgs/OdometryCalculationInfo.h>
#include <arti_base_control/vehicle.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>

namespace arti_base_control
{
class BaseControl
{
public:
  explicit BaseControl(const ros::NodeHandle& private_nh);

protected:
  void reconfigure(BaseControlConfig& config);

  void processVelocityCommand(const geometry_msgs::TwistConstPtr& cmd_vel);
  void processAckermannCommand(const ackermann_msgs::AckermannDriveConstPtr& cmd_ackermann);

  void processOdomTimerEvent(const ros::TimerEvent& event);
  void updateOdometry(
    const ros::Time& time, const geometry_msgs::Twist& velocity,
    arti_base_control_msgs::OdometryCalculationInfo& odometry_calculation_info);
  void publishOdometry(const geometry_msgs::Twist& velocity);

  void publishSupplyVoltage();

  ros::NodeHandle private_nh_;

  BaseControlConfig config_;
  dynamic_reconfigure::Server<BaseControlConfig> reconfigure_server_;

  boost::optional<Vehicle> vehicle_;

  ros::Time odom_update_time_;

  geometry_msgs::Pose2D odom_pose_;

  ros::Publisher odom_pub_;
  boost::optional<tf::TransformBroadcaster> tf_broadcaster_;
  ros::Publisher executed_command_pub_;
  ros::Publisher joint_states_pub_;
  ros::Publisher supply_voltage_pub_;
  ros::Publisher calculation_infos_pub_;

  ros::Subscriber cmd_vel_twist_sub_;
  ros::Subscriber cmd_ackermann_sub_;

  ros::Timer odom_timer_;
};
}


#endif //ARTI_BASE_CONTROL_BASE_CONTROL_H
