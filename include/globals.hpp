#ifndef GLOBALS_HPP
#define GLOBALS_HPP

#include "vex.h"
#include "motions/pistonClass.hpp"
using namespace vex;

template <typename T>
int sgn(T val) { return (T(0) < val) - (val < T(0)); }

vex::distance leftDistanceSensor(PORT21);
vex::distance rightDistanceSensor(PORT1);
vex::distance backDistanceSensor(PORT20);
vex::inertial imu(PORT2);
vex::controller mast(vex::controllerType::primary);
vex::brain Brain;
vex::motor Intake(PORT20, vex::gearSetting::ratio6_1);
Piston clamp(vex::digital_out(Brain.ThreeWirePort.H));

namespace drive
{
  vex::motor leftFront(PORT17, vex::gearSetting::ratio6_1);
  vex::motor leftBack(PORT19, vex::gearSetting::ratio6_1);
  vex::motor leftMid(PORT18, vex::gearSetting::ratio6_1);
  vex::motor rightFront(PORT11, vex::gearSetting::ratio6_1, true);
  vex::motor rightBack(PORT14, vex::gearSetting::ratio6_1, true);
  vex::motor rightMid(PORT15, vex::gearSetting::ratio6_1, true);
  double volt_to_rpm = 600 / 12.0;

  void motorHandler(double volt, int side)
  {
    if (side == 0)
    {
      if (abs(leftMid.velocity(vex::velocityUnits::rpm)) > 450 && sgn(volt) == sgn(leftMid.velocity(vex::velocityUnits::rpm)))
      {
        leftMid.setMaxTorque(0.35, vex::torqueUnits::Nm);
        leftFront.spin(vex::directionType::fwd, volt * volt_to_rpm, vex::velocityUnits::rpm);
        leftBack.spin(vex::directionType::fwd, volt * volt_to_rpm, vex::velocityUnits::rpm);
        leftMid.spin(vex::directionType::fwd, volt, vex::voltageUnits::volt);
      }
      else
      {
        leftMid.setMaxTorque(2.5, vex::currentUnits::amp);
        leftFront.spin(vex::directionType::fwd, volt * volt_to_rpm, vex::velocityUnits::rpm);
        leftBack.spin(vex::directionType::fwd, volt * volt_to_rpm, vex::velocityUnits::rpm);
        leftMid.spin(vex::directionType::fwd, volt, vex::voltageUnits::volt);
      }
    }
    else
    {
      if (abs(rightMid.velocity(vex::velocityUnits::rpm)) > 450 && sgn(volt) == sgn(rightMid.velocity(vex::velocityUnits::rpm)))
      {
        rightMid.setMaxTorque(0.35, vex::torqueUnits::Nm);
        rightFront.spin(vex::directionType::fwd, volt * volt_to_rpm, vex::velocityUnits::rpm);
        rightBack.spin(vex::directionType::fwd, volt * volt_to_rpm, vex::velocityUnits::rpm);
        rightMid.spin(vex::directionType::fwd, volt, vex::voltageUnits::volt);
      }
      else
      {
        rightMid.setMaxTorque(2.5, vex::currentUnits::amp);
        rightFront.spin(vex::directionType::fwd, volt * volt_to_rpm, vex::velocityUnits::rpm);
        rightBack.spin(vex::directionType::fwd, volt * volt_to_rpm, vex::velocityUnits::rpm);
        rightMid.spin(vex::directionType::fwd, volt, vex::voltageUnits::volt);
      }
    }
  }

  void arcade(double power, double turn)
  {
    double l_velocity = power + turn;
    double r_velocity = power - turn;
    motorHandler(l_velocity, 0);
    motorHandler(r_velocity, 1);
  }
} // namespace drive

#endif