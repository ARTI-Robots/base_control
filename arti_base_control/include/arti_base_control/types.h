#ifndef ARTI_BASE_CONTROL_TYPES_H
#define ARTI_BASE_CONTROL_TYPES_H

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace arti_base_control
{
class Axle;

using AxlePtr = std::shared_ptr<Axle>;

class JointSensor;

using JointSensorPtr = std::shared_ptr<JointSensor>;

struct JointState;

using JointStates = std::map<std::string, JointState>;

class JointActuatorFactory;

using JointActuatorFactoryPtr = std::shared_ptr<JointActuatorFactory>;

class PositionControlledJointActuator;

using PositionControlledJointActuatorPtr = std::shared_ptr<PositionControlledJointActuator>;

class Steering;

using SteeringConstPtr = std::shared_ptr<Steering const>;

struct VehicleVelocityConstraint;

using VehicleVelocityConstraints = std::vector<VehicleVelocityConstraint>;

class VelocityControlledJointActuator;

using VelocityControlledJointActuatorPtr = std::shared_ptr<VelocityControlledJointActuator>;

class Wheel;
}

#endif //ARTI_BASE_CONTROL_TYPES_H
