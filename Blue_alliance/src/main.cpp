/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Mark Bowen                                                */
/*    Created:      3/14/2025, 1:09:25 AM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;
brain Brain;
controller Controller1 = controller(primary);
// define your global instances of motors and other devices here
motor LTM = motor(PORT8, ratio6_1, false);  //
motor LBM = motor(PORT13, ratio6_1, true);  // reversed
motor LFM = motor(PORT3, ratio18_1, false); // 5.5W reversed
motor RTM = motor(PORT7, ratio6_1, true);   //
motor RBM = motor(PORT9, ratio6_1, false);  // reversed
motor RFM = motor(PORT6, ratio18_1, true);  // 5.5W revered
inertial Gyro = inertial(PORT5);
optical colorSensor = optical(PORT10);
distance distanceSensor = distance(PORT1);
motor lift11(PORT17, ratio18_1, false);
motor lift55(PORT12, ratio18_1, true);
motor intake(PORT4, ratio36_1, false);
motor arm(PORT16, ratio18_1, true);
pneumatics p1(Brain.ThreeWirePort.A);

int armPosition = 0;
float pi = 3.142;
float dia = 3.25;
float gearRatio = 4 / 3;
vex::thread thread1;
bool killthread1 = false;

void drive(int left, int right, int wt)
{
  LTM.spin(forward, left, pct);
  LBM.spin(forward, left, pct);
  LFM.spin(forward, left, pct);
  RTM.spin(forward, right, pct);
  RBM.spin(forward, right, pct);
  RFM.spin(forward, right, pct);
  wait(wt, msec);
}

void driveBrake()
{
  LTM.stop(brake);
  LBM.stop(brake);
  LFM.stop(brake);
  RTM.stop(brake);
  RBM.stop(brake);
  RFM.stop(brake);
}

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

void intakeForward()
{
  lift(100);
  intake.spin(reverse, 100, percent);
}

void intakeStop()
{
  liftStop();
  intake.stop();
}

void intakeReverse()
{
  lift(-100);
}

void clampOpen()
{
  p1.set(false);
}

void clampClose()
{
  p1.set(true);
}

void liftSet(float target, int speed)
{
  float x = 0;
  lift11.setPosition(0, deg);
  x = lift11.position(deg);

  if (target >= 0)
  { // if your target is greater than 0 we will drive forward
    while (x <= target)
    {
      lift(speed);
      x = lift11.position(deg);
      Brain.Screen.printAt(10, 20, "degrees = %0.2f", x);
    }
  }
  else if (target < 0)
  {
    while (x >= target)
    { // target less than 0 the robot will drive backward
      lift(-speed);
      x = lift11.position(deg);
      Brain.Screen.printAt(10, 20, "degrees = %0.2f", x);
    }
  }

  lift(0);
}

void armMiddlePosition()
{
  int disc = 0;
  arm.setPosition(0, deg);
  arm.spinToPosition(225, degrees, 50, velocityUnits::pct);
  arm.setBrake(brakeType::hold);
  armPosition = 1;
  intake.spin(reverse, 100, percent);
  /*while (!killthread1)
  {
    double distance = distanceSensor.objectDistance(mm);
    lift(-25);
    if (distance < 100 && distance > 80)
    {
      lift(0);
      liftSet(-160);
      intake.spin(reverse, 100, percent);
      break;
    }
    vex::this_thread::sleep_for(10);
  }*/
  while (!killthread1)
  {
    double distance = distanceSensor.objectDistance(mm);
    if (distance < 50)
    {
      liftSet(-320,50);
      intake.stop();
      break;
    }
    vex::this_thread::sleep_for(10);
  }
  killthread1 = false;
}

void armSet(float target, int timeout_ms)
{
  float x = 0;
  arm.setPosition(0, deg);
  x = arm.position(deg);

  vex::timer t; // Initialize a timer
  t.clear();    // Reset the timer

  if (target >= 0)
  {
    while (x <= target)
    {
      if (t.time() > timeout_ms)
      {
        Brain.Screen.printAt(10, 120, "Timeout reached while moving arm forward.");
        break;
      }
      arm.spin(forward, 100, percent);
      x = arm.position(deg);
    }
  }
  else if (target <= 0)
  {
    while (x >= target)
    {
      if (t.time() > timeout_ms)
      {
        Brain.Screen.printAt(10, 120, "Timeout reached while moving arm reverse.");
        break;
      }
      arm.spin(reverse, 100, percent);
      x = arm.position(deg);
    }
  }
  arm.stop();
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

void wallStake()
{
  armSet(600, 1000);
  wait(10, msec);
  liftSet(-600,100);
}

void increaseArmPosition()
{
  Brain.Screen.clearScreen();
  if (killthread1)
  {
    Brain.Screen.printAt(10, 10, "Thread is being killed, can't increase position");
    return;
  }
  if (armPosition == 0)
  {
    Brain.Screen.printAt(10, 10, "Moving to middle position...");
    killthread1 = false;
    thread1 = vex::thread(armMiddlePosition);
    wait(10, msec);
  }
  else if (armPosition == 1)
  {
    Brain.Screen.printAt(10, 10, "Moving to top position...");
    wallStake();
    armPosition = 2;
    wait(10, msec);
  }
  else
  {
    Brain.Screen.printAt(10, 10, "Already at top position");
    Brain.Screen.printAt(10, 100, "armPosition = %d", armPosition);
  }
}

void bottomPosition()
{
  if (armPosition == 1)
  {
    killthread1 = true;
    armSet(-250, 1000);
    arm.setBrake(brakeType::hold);
    lift(0);
  }
  if (armPosition == 2)
  {
    armSet(-900, 1000);
    arm.setBrake(brakeType::hold);
    lift(0);
  }
  else
  {
    armSet(-50,250);
    arm.setBrake(brakeType::hold);
  }
  armPosition = 0;
  
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
  //armSet(-150,500);
  arm.setBrake(brakeType::hold);

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

  armSet(-50,50);

  //beginning of match auton
/*clampOpen();
inchDrive(-20);
clampClose();
liftSet(360,100);
//if we go further
gyroTurnLeft(120);//turn right for red
clampOpen();
inchDrive(20);
clampClose();
inchDrive(-20);
gyroTurnLeft(135);//calibrate so intake faces ladder
inchDrive(15);*/
//end of match auton

//beginning of skills auton: changed but would work for 2 disc score on hihstks
/*clampOpen();
inchDrive(-20);//change distance
clampClose();
lift(100);
//full stake after preload
gyroTurnRight(120);//turn right for red
inchDrive(20);//change distance, inbetween ladder corner and wall stake
gyroTurnRight(60);
inchDrive(48);
inchDrive(-10);
inchDrive(12);
gyroTurnLeft(180);
inchDrive(96)
driveBrake();
wait(100,msec);
inchDrive(24);
inchDrive(-10);//change distance
gyroTurnLeft(90);//calibrate to face directly at wall to park
inchDrive(-24);
//wall stake
driveBrake();//needed?
wait(100,msec);
inchDrive(10);//may need wait aftr drive to use gyro turn correctly
gyroTurnLeft(100);//change angle?
inchDrive(24);
wait(100,msec);
inchDrive()*/

clampOpen();
  inchDrive(-16);
  clampClose();
  gyroTurnRight(225);
  inchDrive(15);
  intakeStart(); // first disk
  // second disk
  inchDrive(29);
  driveBrake();
  wait(1000, msec);
  inchDrive(-2);
  driveBrake();
  gyroTurnLeft(90);
  inchDrive(30);
  driveBrake();
  // third disk
  wait(500, msec);
  gyroTurnLeft(90);
  inchDrive(12);
  driveBrake();
  wait(200, msec);
  inchDrive(12);
  driveBrake();
  wait(500, msec);
  inchDrive(8);
  driveBrake();
  wait(200, msec);
  inchDrive(-2);
  wait(200,msec)
  gyroTurnRight(135);
  driveBrake();
  inchDrive(15);
  driveBrake();
  wait(1000, msec);
  gyroTurnRight(45);
  inchDrive(-12);
  wait(500, msec);
  clampOpen();
  intakeStop();
  increase_arm_position();

  inchDrive(12);
  gyroTurnRight(8);
  inchDrive(146);
  gyroTurnRight(10);
  inchDrive(100);


  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
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


  clampOpen();
  //armSet(-150,100);
  arm.setBrake(brakeType::hold);
  // User control code here, inside the loop
  armPosition = 0;
  while (1)
  {
    // readIMU(); // print direction we are facing
    int left = Controller1.Axis3.position();
    int right = Controller1.Axis2.position();
    drive(left, right, 10);
    Controller1.ButtonR1.pressed(intakeForward);
    Controller1.ButtonR2.pressed(intakeReverse);
    Controller1.ButtonL1.pressed(clampOpen);
    Controller1.ButtonL2.pressed(clampClose);
    Controller1.ButtonX.pressed(intakeStop);
    Controller1.ButtonUp.pressed(increaseArmPosition);
    Controller1.ButtonDown.pressed(bottomPosition);
    Brain.Screen.printAt(10, 100, "armPosition = %d", armPosition);
    //int discColor = colorSensor.color();

    /*if (armPosition == 0  && discColor>=100 && discColor<=150)
    {
      lift(0);
      wait(20,msec);
      lift(100);
    }*/

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
