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
#include <vector>

using namespace vex;

// A global instance of competition
competition Competition;
brain Brain;
controller Controller1 = controller(primary);

//Pretty sure this the correct motor setup
motor LeftMotor1 = motor(PORT11, ratio6_1, true);
motor LeftMotor2 = motor(PORT4, ratio6_1, false);
motor LeftMotor3 = motor(PORT5, ratio6_1, true);
motor RightMotor1 = motor(PORT6, ratio6_1, false);
motor RightMotor2 = motor(PORT7, ratio6_1, true);
motor RightMotor3 = motor(PORT8, ratio6_1, false);
pneumatics Pneumatics = pneumatics(Brain.ThreeWirePort.G);
motor Intake = motor(PORT17, ratio36_1, false);
motor Lift = motor(PORT18, ratio18_1, false);
distance DistanceSensor1 = distance(PORT9);
distance DistanceSensor2 = distance(PORT10);


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

int MaxSpeed(int speed){
  if (speed >= 80){
    return 80;
  }
  else {
    return speed;
  }
}


void drive(int left, int right, int wt) {
  RightMotor1.spin(forward, right, percent);
  RightMotor2.spin(forward, right, percent);
  RightMotor3.spin(forward, right, percent);
  wait(1,msec);
  LeftMotor1.spin(forward, left, percent);
  LeftMotor2.spin(forward, left, percent);
  LeftMotor3.spin(forward, left, percent);
  wait(wt,msec);
}

void driveBrake(){
  LeftMotor1.stop();
  LeftMotor2.stop();  
  LeftMotor3.stop();
  wait(1,msec);
  RightMotor1.stop();
  RightMotor2.stop();
  RightMotor3.stop();
  
}
  


void turnLeft(int right, int wt) {
  RightMotor1.spin(forward, right, percent);
  RightMotor2.spin(forward, right, percent);
  RightMotor3.spin(forward, right, percent);
}

void intake_start() {
  checkMotorTorque(Lift);
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
  Pneumatics.set(true);
}
void clamp_close() {
  Pneumatics.set(false);
}

float pi=3.142;
float dia=3.25;
float gearRatio = 4/3;

void inchDrive(float target) {
float x=0;
LeftMotor1.setPosition(0,rev);
x = LeftMotor1.position(rev)*pi*dia*gearRatio;

if (target >= 0 ){ //if your target is greater than 0 we will drive forward 
  while (x <= target ) { 
  drive(35, 35, 10);
  //use gyro here
  x = LeftMotor1.position(rev)*dia*pi*gearRatio; Brain.Screen.printAt(10, 20, "inches = %0.2f", x); 
  } 
} 
else if (target < 0){ 
    while (x >= target){ //target less than 0 the robot will drive backward
      drive(-35, -35, 10); 
      x = LeftMotor1.position(rev)*dia*pi*gearRatio; Brain.Screen.printAt(10, 20, "inches = %0.2f", x);
    }
} 
drive(0, 0, 0); 
}

void turn_right(int target){
  float x=0;
  LeftMotor1.setPosition(0,rev);
  x = LeftMotor1.position(rev)*pi*dia*gearRatio;

  if (target >= 0 ){ //if your target is greater than 0 we will drive forward 
    while (x <= target ) { 
    drive(35, -35, 10);
    //use gyro here
    x = LeftMotor1.position(rev)*dia*pi*gearRatio; Brain.Screen.printAt(10, 20, "inches = %0.2f", x); 
  } 
  } 
}

void turn_left(int target) {
  float x=0;
  RightMotor1.setPosition(0,rev);
  x=RightMotor1.position(rev)*pi*dia*gearRatio;

  if (target >= 0 ){ //if your target is greater than 0 we will drive forward 
    while (x <= target ) { 
    drive(-35, 35, 10);
    //use gyro here
    x = RightMotor1.position(rev)*dia*pi*gearRatio; Brain.Screen.printAt(10, 20, "inches = %0.2f", x); 
  } 
  } 
}

void reverse_right_motor(int target){
  float x=0;
  RightMotor1.setPosition(0,rev);
  x=RightMotor1.position(rev)*pi*dia*gearRatio;
    while (x >= target ) { 
    drive(0, -35, 10);
    //use gyro here
    x = RightMotor1.position(rev)*dia*pi*gearRatio; Brain.Screen.printAt(10, 20, "inches = %0.2f", x); 
  } 
}
  
void print_distance(){
  float dist1 = DistanceSensor1.objectDistance(inches);
  float dist2 = DistanceSensor2.objectDistance(inches);

  Brain.Screen.print("dist1,dist2");

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
  clamp_open();
  inchDrive(-16);
  clamp_close();
  turn_right(41);
  inchDrive(15);
  intake_start(); //first disk
  //second disk
  inchDrive(29);
  driveBrake();
  wait(2000,msec);
  inchDrive(-2);
  driveBrake();
  turn_left(15);
  inchDrive(30);
  driveBrake();
  //third disk
  wait(1000,msec);
  turn_left(15);
  inchDrive(12);
  driveBrake();
  wait(500,msec);
  inchDrive(12);
  driveBrake();
  wait(1000,msec);
  inchDrive(8);
  driveBrake();
  wait(500,msec);
  reverse_right_motor(-35);
  driveBrake();
  inchDrive(15);
  driveBrake();
  wait(4000,msec);
  turn_right(20);
  inchDrive(-12);
  wait(1000,msec);
  clamp_open();
  intake_stop();
  inchDrive(12);
  turn_right(8);
  inchDrive(146);
  turn_right(10);
  inchDrive(100);
  


 /* clamp_open();
  inchDrive(-16);
  clamp_close();
  turn_left(41);
  inchDrive(15);
  intake_start(); //first disk
  //second disk
  inchDrive(29);
  driveBrake();
  wait(2000,msec);
  inchDrive(-2);
  driveBrake();
  turn_right(15);
  inchDrive(30);
  driveBrake();
  //third disk
  wait(1000,msec);
  turn_left(15);
  inchDrive(12);
  driveBrake();
  wait(500,msec);
  inchDrive(12);
  driveBrake();
  wait(1000,msec);
  inchDrive(8);
  driveBrake();
  wait(500,msec);
  reverse_right_motor(-35);
  driveBrake();
  inchDrive(15);
  driveBrake();
  wait(4000,msec);
  turn_left(25);
  inchDrive(-12);
  wait(2000,msec);
  clamp_open();
  inchDrive(12);
  turn_left(10);*/




  
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
  //autonomous(); //delete later
  clamp_open();
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
    left = MaxSpeed(left);
    right = MaxSpeed(right);
    drive(left, right, 10);

    Controller1.ButtonR1.pressed(intake_start);
    Controller1.ButtonR2.pressed(intake_reverse);
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
