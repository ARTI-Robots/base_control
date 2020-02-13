#ifndef ARTI_BASE_CONTROL_VESC_VESCMOTOR_H
#define ARTI_BASE_CONTROL_VESC_VESCMOTOR_H

#include <atomic>
#include <chrono>
#include <ros/node_handle.h>
#include <vesc_driver/types.h>
#include <arti_base_control/abstract_motor.h>

namespace arti_base_control_vesc
{
class VescMotor : public arti_base_control::AbstractMotor
{
public:
  VescMotor(const ros::NodeHandle& private_nh, const vesc_driver::DriverFactoryPtr& driver_factory,
            const std::chrono::duration<double>& execution_duration);
  virtual ~VescMotor() = default;

  /**
   * Gets the motor controller's supply voltage in V.
   * @return the supply voltage in V.
   */
  double getMotorSupplyVoltage();

protected:
  void createDriver();
  virtual void processMotorControllerState(const vesc_driver::MotorControllerState& state) = 0;

  ros::NodeHandle private_nh_;
  vesc_driver::VescDriverInterfacePtr driver_;

private:
  void callProcessMotorControllerState(const vesc_driver::MotorControllerState& state);

  vesc_driver::DriverFactoryPtr driver_factory_;
  std::chrono::duration<double> execution_duration_;
  std::atomic<double> supply_voltage_;
};

typedef std::shared_ptr<VescMotor> VescMotorPtr;
}

#endif //ARTI_BASE_CONTROL_VESC_VESCMOTOR_H
