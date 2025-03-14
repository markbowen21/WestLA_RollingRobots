/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       markb                                                     */
/*    Created:      3/1/2025, 6:37:18 PM                                      */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "math.h"

using namespace vex;

// A global instance of competition
competition Competition;
brain Brain;
controller Controller1 = controller(primary);
motor LTM = motor(PORT8, ratio6_1, false);  //
motor LBM = motor(PORT13, ratio6_1, true);  // reversed
motor LFM = motor(PORT3, ratio18_1, false); // 5.5W reversed
motor RTM = motor(PORT7, ratio6_1, true);   //
motor RBM = motor(PORT9, ratio6_1, false);  // reversed
motor RFM = motor(PORT6, ratio18_1, true);  // 5.5W revered

inertial Gyro = inertial(PORT5);
//optical Color = optical(PORT4);
distance Distance1 = distance(PORT1);
//distance Distance2 = distance(PORT16);

motor lift11(PORT17, ratio18_1, false);
motor lift55(PORT12, ratio18_1, true);
motor intake(PORT4, ratio36_1, false);
motor arm(PORT16, ratio18_1, false);

pneumatics p1(Brain.ThreeWirePort.A);

// define your global instances of motors and other devices here
// We need flags to keep track of the state of the robot
// 0 = down position , 1 = middle position, 2 = top position
int arm_position = 0;
// define your functions here


void drive(int left, int right, int wt)
{
  RTM.spin(forward, right, percent);
  RBM.spin(forward, right, percent);
  RFM.spin(forward, right, percent);
  LTM.spin(forward, left, percent);
  LBM.spin(forward, left, percent);
  LFM.spin(forward, left, percent);
  wait(wt, msec);
}

void driveBrake()
{
  RTM.stop();
  RBM.stop();
  RFM.stop();
  LTM.stop();
  LBM.stop();
  LFM.stop();
}

void gyroPrint()
{
  float heading = Gyro.rotation(deg);
  Brain.Screen.printAt(1, 60, "heading  =  %.2f. degrees", heading);
}

void gyroTurnRight(float target)
{
  float heading = 0.0;            // initialize a variable for heading
  Gyro.setRotation(0.0, degrees); // reset Gyro to zero degrees
  while (heading <= target)
  {
    drive(50, -50, 10);           // turn right at half speed
    heading = Gyro.rotation(deg); // measure the heading of the robot
  }
  drive(0, 0, 0); // stop the drive
}

void gyroTurnLeft(float target)
{
  float heading = 0.0;            // initialize a variable for heading
  Gyro.setRotation(0.0, degrees); // reset Gyro to zero degrees
  while (heading <= target)
  {
    drive(-50, 50, 10);           // turn right at half speed
    heading = Gyro.rotation(deg); // measure the heading of the robot
  }
  drive(0, 0, 0); // stop the drive
}

void lift(int speed)
{
  lift11.spin(forward, speed, percent);
  lift55.spin(forward, speed, percent);
}

void liftStop()
{
  lift11.stop();
  lift55.stop();
}

void intakeStart()
{

  lift(100);
  intake.spin(reverse, 100, percent);
}

void intakeStop()
{
  lift(0);
  intake.stop();
}

void intakeReverse(){
  lift(-100);
}

void clampOpen()
{
  p1.set(true);
}

void clampClose()
{
  p1.set(false);
}

float pi = 3.142;
float dia = 3.25;
float gearRatio = 4 / 3;

void increaseArm_Position()
{
  /*if (arm_position = 0)
  {
    armSet(20);
  }
  if (arm_position = 1)
  {
    intakeStop();
    armSet(40);
    wait(200, msec);

    lift(100);
    wait(200, msec);
    armSet(-40);
    arm_position--;
    // going to top position should dump rings automatically
  }*/
  if (arm_position >= 1)
  {
    arm_position++;
  }
  else if (arm_position = 2)
  {
    arm_position = 0;
  }
}
void decreaseArm_Position()
{
  /*if (arm_position = 1)
  {
    armSet(-20);
  }
  if (arm_position = 2)
  {
    armSet(-40);
  }*/
  arm_position--;
}

void armSet(float target)
{
  float x = 0;
  arm.setPosition(0, deg);
  x = arm.position(deg);

  if (target >= 0)
  {
    while (x <= target)
    {
      arm.spin(forward, 100, percent);

      x = arm.position(deg); /*Brain.Screen.printAt(10, 20, "degrees = %0.2f", x);*/
    }
  }
  else if (target <= 0)
  {
    while (x >= target)
    {
      arm.spin(reverse, 100, percent);

      x = arm.position(deg); /*Brain.Screen.printAt(10, 20, "degrees = %0.2f", x);*/
    }
  }
  arm.stop();
}

void liftSet(float target)
{
  float x = 0;
  lift11.setPosition(0, deg);
  x = lift11.position(deg) * pi * dia * gearRatio;

  if (target >= 0)
  { // if your target is greater than 0 we will drive forward
    while (x <= target)
    {
      lift(50);
      // use gyro here
      x = lift11.position(deg) * dia * pi * gearRatio;
      Brain.Screen.printAt(10, 20, "inches = %0.2f", x);
    }
  }
  else if (target < 0)
  {
    while (x >= target)
    { // target less than 0 the robot will drive backward
      lift(-50);
      x = lift11.position(deg) * dia * pi * gearRatio;
      Brain.Screen.printAt(10, 20, "inches = %0.2f", x);
    }
  }

  lift(0);
}

// bool x;
/*void stakeMode()
{
  bool x = true;
  while (x = true)
  {

    float discs = 0;
    lift11.setPosition(0, rev);
    lift(50);
    if (discs = 0 && distance <= 15 && distance >= 50)
    {
      // intakeStop();
      lift(0);
      liftSet(-80);
      armSet(15);
      intakeStart();
    }

    // when distance<=110 && distance>=90 rotate 160degrees,discs++

    if (discs <= 2 && distance <= 15 && distance >= 50)
    {
      liftSet(-160);
      discs++;
    }
    else if (discs = 2)
    {
      armSet(30);
      intakeStop();
      discs = 3;
    }
  }
}

void loadStake()
{

  lift(-100);
  wait(4000, msec);
  lift(0);
  inchDrive(20);
  armSet(-45);
}*/

void inchDrive(float target)
{
  float x = 0;
  LFM.setPosition(0, rev);
  x = LFM.position(rev) * pi * dia * gearRatio;

  if (target >= 0)
  { // if your target is greater than 0 we will drive forward
    while (x <= target)
    {
      drive(35, 35, 10);
      // use gyro here
      x = LFM.position(rev) * dia * pi * gearRatio;
      Brain.Screen.printAt(10, 20, "inches = %0.2f", x);
    }
  }
  else if (target < 0)
  {
    while (x >= target)
    { // target less than 0 the robot will drive backward
      drive(-35, -35, 10);
      x = LFM.position(rev) * dia * pi * gearRatio;
      Brain.Screen.printAt(10, 20, "inches = %0.2f", x);
    }
  }
  drive(0, 0, 0);
}

void readIMU()
{
  double heading20 = 0;
  double pitch20 = 0;
  double roll20 = 0;
  double yaw20 = 0;
  double rotation20 = 0;
  double ax = 0;
  double ay = 0;
  double az = 0;

  heading20 = Gyro.heading(degrees); // why did we add this? we didnt use it for inchdrive
  pitch20 = Gyro.orientation(pitch, degrees);
  roll20 = Gyro.orientation(roll, degrees);
  yaw20 = Gyro.orientation(yaw, degrees);
  rotation20 = Gyro.rotation(degrees);
  ax = Gyro.acceleration(xaxis);
  ay = Gyro.acceleration(yaxis);
  az = Gyro.acceleration(zaxis);

  Brain.Screen.clearScreen();
  Brain.Screen.printAt(1, 40, "IMU values");
  Brain.Screen.printAt(1, 40, "Heading = %.2f  Degrees", heading20);
  Brain.Screen.printAt(1, 60, "Pitch = %.2f Degrees", pitch20);
  Brain.Screen.printAt(1, 80, "Roll= %.2f  Degrees ", roll20);
  Brain.Screen.printAt(1, 100, "Yaw = %.2f  Degrees ", yaw20);
  Brain.Screen.printAt(1, 120, "rotation = %.2f  Degrees ", rotation20);
  Brain.Screen.printAt(1, 140, "X accel = %.2f", ax);
  Brain.Screen.printAt(1, 160, "Y accel = %.2f", ay);
  Brain.Screen.printAt(1, 180, "Z accel = %.2f", az);
  wait(20, msec);
}

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void)
{

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...

  while (Gyro.isCalibrating())
  { // Wait for Gyro Calibration , Sleep but Allow other tasks to run
    this_thread::sleep_for(20);
  }
  while (true)
  {
    readIMU();
    this_thread::sleep_for(50);
  }
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

void autonomous(void)
{
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
  /*clampOpen();
  inchDrive(-16);
  clampClose();
  gyroTurnRight(41);
  inchDrive(15);
  intakeStart(); // first disk
  // second disk
  inchDrive(29);
  driveBrake();
  wait(2000, msec);
  inchDrive(-2);
  driveBrake();
  gyroTurnLeft(15);
  inchDrive(30);
  driveBrake();
  // third disk
  wait(1000, msec);
  gyroTurnLeft(15);
  inchDrive(12);
  driveBrake();
  wait(500, msec);
  inchDrive(12);
  driveBrake();
  wait(1000, msec);
  inchDrive(8);
  driveBrake();
  wait(500, msec);
  reverse_right_motor(-35);
  driveBrake();
  inchDrive(15);
  driveBrake();
  wait(4000, msec);
  gyroTurnRight(20);
  inchDrive(-12);
  wait(1000, msec);
  clampOpen();
  intakeStop();
  inchDrive(12);
  gyroTurnRight(8);
  inchDrive(146);
  gyroTurnRight(10);
  inchDrive(100);*/
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

void usercontrol(void)
{
  // User control code here, inside the loop
  while (1)
  {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................
    readIMU(); // print direction we are facing
    int left = Controller1.Axis3.position();
    int right = Controller1.Axis2.position();
    drive(left, right, 10);
    Controller1.ButtonUp.pressed(increaseArm_Position);
    Controller1.ButtonDown.pressed(decreaseArm_Position);
    Controller1.ButtonR1.pressed(intakeStart);
    Controller1.ButtonR2.pressed(intakeStop);
    Controller1.ButtonL1.pressed(clampClose);
    Controller1.ButtonL2.pressed(clampOpen);
    Controller1.ButtonA.pressed(intakeReverse);

    float distance = Distance1.objectDistance(mm);

    if (arm_position = 1)
    {
      //lift(50);

      if (distance >= 90 && distance <= 110)
      {
        // intakeStop();
        lift(0);
        liftSet(-160);
        armSet(15);
        intakeStart();
      }

      // when distance<=110 && distance>=90 rotate 160degrees,discs++

      if (distance <= 15 && distance >= 50)
      {
        liftSet(-320);
      }
    }
    else if (arm_position = 2)
    {
      armSet(30);
      intakeStop();
      arm_position = 0;
    }
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main()
{
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true)
  {
    wait(100, msec);
  }
}
