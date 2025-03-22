#ifndef GLOBALS_HPP
#define GLOBALS_HPP

#include "vex.h"
using namespace vex;

template <typename T> int sgn(T val) { return (T(0) < val) - (val < T(0)); }

vex::distance leftDistanceSensor(PORT21);
vex::distance rightDistanceSensor(PORT1);
vex::distance backDistanceSensor(PORT20);
vex::inertial imu(PORT2);
vex::controller mast(vex::controllerType::primary);
vex::brain Brain;

namespace drive {
vex::motor leftFront(PORT12);
vex::motor leftBack(PORT11);
vex::motor rightFront(PORT13, vex::gearSetting::ratio18_1, true);
vex::motor rightBack(PORT14, vex::gearSetting::ratio18_1, true);
void arcade(double power, double turn) {
  leftFront.spin(vex::directionType::fwd, power + turn, vex::voltageUnits::volt);
  leftBack.spin(vex::directionType::fwd, power + turn, vex::voltageUnits::volt);
  rightFront.spin(vex::directionType::fwd, power - turn, vex::voltageUnits::volt);
  rightBack.spin(vex::directionType::fwd, power - turn, vex::voltageUnits::volt);
}
} 

#endif