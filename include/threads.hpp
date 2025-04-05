#ifndef THREADS_HPP
#define THREADS_HPP
#include "globals.hpp"
#include "odom/localization.hpp"
#include "vex.h"
#include <iostream>

void odom() {
  ParticleFilter pf(2000,
                    {{-5.75f, -0.5f, 90.0f},
                     {5.75f, -0.85f, 270.0f},
                     {-1.25f, -4.15f, 180.0f}},
                    0, 0, 0.5, 0.25);
  float prev_right = drive::rightFront.position(vex::rotationUnits::rev);
  float prev_left = drive::leftFront.position(vex::rotationUnits::rev);
  float prev_heading = imu.rotation(vex::rotationUnits::deg);
  float ROT_TO_DISTANCE = 4 * PI;
  while (true) {
    Brain.Screen.clearScreen();
    float heading = imu.angle(vex::rotationUnits::deg);
    float leftDistance =
        leftDistanceSensor.objectDistance(vex::distanceUnits::in);
    float rightDistance =
        rightDistanceSensor.objectDistance(vex::distanceUnits::in);
    float backDistance =
        backDistanceSensor.objectDistance(vex::distanceUnits::in);
    float measurements[3] = {leftDistance, rightDistance, backDistance};

    if (leftDistanceSensor.objectRawSize() < 60) {
      measurements[0] = 9999;
    }

    if (rightDistanceSensor.objectRawSize() < 60) {
      measurements[1] = 9999;
    }
    if (backDistanceSensor.objectRawSize() < 60) {
      measurements[2] = 9999;
    }

    float right = drive::rightFront.position(vex::rotationUnits::rev);
    float left = drive::leftFront.position(vex::rotationUnits::rev);
    float delta_right = right - prev_right;
    float delta_left = left - prev_left;
    float delta_heading = heading - prev_heading;

    if (delta_heading < -180) {
      delta_heading += 360;
    } else if (delta_heading > 180) {
      delta_heading -= 360;
    }
    int start = Brain.Timer.time(msec);
    pf.update(heading * DEG_TO_RAD, delta_heading * DEG_TO_RAD,
              delta_left * ROT_TO_DISTANCE, delta_right * ROT_TO_DISTANCE,
              measurements);
    int end = Brain.Timer.time(msec);
    // std::cout << end - start << std::endl;
    prev_right = right;
    prev_left = left;
    prev_heading = heading;

    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Estimate: %f, %f", pf.estimate().first,
                       pf.estimate().second);

    vex::task::sleep(10);
  }
}

#endif