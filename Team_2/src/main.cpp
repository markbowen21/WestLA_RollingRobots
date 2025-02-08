/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       mark-bowen                                                */
/*    Created:      2/7/2025, 4:26:30 PM                                      */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include <cmath>

using namespace vex;

// A global instance of competition
competition Competition;
brain Brain;
controller Controller1 = controller(primary);
motor LeftMotor1 = motor(PORT3, ratio18_1, false);
motor LeftMotor2 = motor(PORT4, ratio18_1, false);
motor LeftMotor3 = motor(PORT5, ratio18_1, false);
motor RightMotor1 = motor(PORT6, ratio18_1, true);
motor RightMotor2 = motor(PORT7, ratio18_1, true);
motor RightMotor3 = motor(PORT8, ratio18_1, true);
pneumatics Pneumatics = pneumatics(Brain.ThreeWirePort.A);
motor Intake = motor(PORT17, ratio18_1, false);
motor Lift = motor(PORT18, ratio6_1, false);

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void drive(int left, int right) {
  LeftMotor1.spin(forward, left, percent);
  LeftMotor2.spin(forward, left, percent);
  LeftMotor3.spin(forward, left, percent);
  RightMotor1.spin(forward, right, percent);
  RightMotor2.spin(forward, right, percent);
  RightMotor3.spin(forward, right, percent);
}

void intake_start() {
  Intake.spin(forward, 100, percent);
  Lift.spin(forward, 100, percent);
}
void intake_reverse() {
  Intake.spin(reverse, 100, percent);
  Lift.spin(reverse, 100, percent);
}
void intake_stop() {
  Intake.stop();
  Lift.stop();
}

void clamp_open() {
  Pneumatics.set(false);
}
void clamp_close() {
  Pneumatics.set(true);
}

void pre_auton(void) {

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
    double wheelDiameterInches = 4.0;
  double wheelCircumferenceInches = wheelDiameterInches * M_PI;
  double distanceInFeet = 2.0;
  double distanceInInches = distanceInFeet * 12.0;
  double rotations = distanceInInches / wheelCircumferenceInches;

  // Adjust for the gear ratio
  double adjustedRotations = rotations * (4.0 / 3.0);

  // Move forward
  LeftMotor1.spinFor(adjustedRotations, rotationUnits::rev, 50, velocityUnits::pct, false);
  LeftMotor2.spinFor(adjustedRotations, rotationUnits::rev, 50, velocityUnits::pct, false);
  LeftMotor3.spinFor(adjustedRotations, rotationUnits::rev, 50, velocityUnits::pct, false);
  RightMotor1.spinFor(adjustedRotations, rotationUnits::rev, 50, velocityUnits::pct, false);
  RightMotor2.spinFor(adjustedRotations, rotationUnits::rev, 50, velocityUnits::pct, false);
  RightMotor3.spinFor(adjustedRotations, rotationUnits::rev, 50, velocityUnits::pct, true);

  // Close the clamp
  clamp_close();
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................
    int left = Controller1.Axis3.position();
    int right = Controller1.Axis2.position();
    drive(left, right);

    Controller1.ButtonR2.pressed(intake_start);
    Controller1.ButtonR1.pressed(intake_reverse);
    Controller1.ButtonX.pressed(intake_stop);

    Controller1.ButtonL1.pressed(clamp_open);
    Controller1.ButtonL2.pressed(clamp_close);

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
