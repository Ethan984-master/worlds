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
vex::motor leftMid(PORT19);
vex::motor rightFront(PORT13, vex::gearSetting::ratio18_1, true);
vex::motor rightBack(PORT14, vex::gearSetting::ratio18_1, true);
vex::motor rightMid(PORT17, vex::gearSetting::ratio18_1, true);
double volt_to_rpm = 0.06;

void motorHandler(double rpm, int side) {
  if (abs(rpm) < 75) {
    if (side == 0) {
      leftMid.setMaxTorque(2.1, vex::torqueUnits::Nm);

      leftFront.spin(vex::directionType::fwd, rpm * volt_to_rpm,
                     vex::velocityUnits::rpm);
      leftBack.spin(vex::directionType::fwd, rpm * volt_to_rpm,
                    vex::velocityUnits::rpm);
      leftMid.spin(vex::directionType::fwd, rpm, vex::voltageUnits::volt);
    } else {
      rightMid.setMaxTorque(2.1, vex::torqueUnits::Nm);

      rightFront.spin(vex::directionType::fwd, rpm * volt_to_rpm,
                      vex::velocityUnits::rpm);
      rightBack.spin(vex::directionType::fwd, rpm * volt_to_rpm,
                     vex::velocityUnits::rpm);
      rightMid.spin(vex::directionType::fwd, rpm, vex::voltageUnits::volt);
    }
  } else {
    if (side == 0) {
      leftMid.setMaxTorque(1, vex::torqueUnits::Nm);

      leftFront.spin(vex::directionType::fwd, rpm * volt_to_rpm,
                     vex::velocityUnits::rpm);
      leftBack.spin(vex::directionType::fwd, rpm * volt_to_rpm,
                    vex::velocityUnits::rpm);
      leftMid.spin(vex::directionType::fwd, rpm, vex::voltageUnits::volt);
    } else {
      rightMid.setMaxTorque(1, vex::torqueUnits::Nm);

      rightFront.spin(vex::directionType::fwd, rpm * volt_to_rpm,
                      vex::velocityUnits::rpm);
      rightBack.spin(vex::directionType::fwd, rpm * volt_to_rpm,
                     vex::velocityUnits::rpm);
      rightMid.spin(vex::directionType::fwd, rpm, vex::voltageUnits::volt);
    }
  }
}

void arcade(double power, double turn) {
  double l_velocity = power + turn;
  double r_velocity = power - turn;
  motorHandler(l_velocity, 0);
  motorHandler(r_velocity, 1);
}
} // namespace drive

#endif