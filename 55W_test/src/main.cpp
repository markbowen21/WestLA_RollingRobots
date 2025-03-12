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
motor LTM = motor(PORT8, ratio6_1, false); //
motor LBM = motor(PORT3, ratio6_1, true); // reversed
motor LFM = motor(PORT13, ratio18_1, false); //5.5W reversed
motor RTM = motor(PORT7, ratio6_1, true); //
motor RBM = motor(PORT17, ratio6_1, false); //reversed
motor RFM = motor(PORT6, ratio18_1, true); //5.5W revered 

inertial Gyro = inertial(PORT12);
optical Color = optical(PORT4);
optical Distance1 = optical(PORT11);
optical Distance2 = optical(PORT16);


motor lift11(PORT5,ratio18_1,false);
motor lift55(PORT9,ratio18_1,true);
motor intake(PORT10,ratio36_1,true);
motor arm(PORT7,ratio18_1,false);

pneumatics p1(Brain.ThreeWirePort.A);





// define your global instances of motors and other devices here






// define your functions here
void drive(int left, int right, int wt) {
  RTM.spin(forward, right, percent);
  RBM.spin(forward, right, percent);
  RFM.spin(forward, right, percent);
  LTM.spin(forward, left, percent);
  LBM.spin(forward, left, percent);
  LFM.spin(forward, left, percent);
  wait(wt,msec);
}

void drivebrake() {
  RTM.stop();
  RBM.stop();
  RFM.stop();
  LTM.stop();
  LBM.stop();
  LFM.stop();
}

void gyroPrint() {
  float heading = Gyro.rotation(deg);
  Brain.Screen.printAt(1, 60, "heading  =  %.2f. degrees", heading);
}

void gyroTurnRight(float target) {
  float heading=0.0; //initialize a variable for heading
  Gyro.setRotation(0.0, degrees);  //reset Gyro to zero degrees
  while(heading<=target){
    drive(50, -50, 10); //turn right at half speed
    heading=Gyro.rotation(deg);  //measure the heading of the robot
  }
  drive(0, 0, 0);  //stop the drive
}

void gyroTurnLeft(float target) {
  float heading=0.0; //initialize a variable for heading
  Gyro.setRotation(0.0, degrees);  //reset Gyro to zero degrees
  while(heading<=target){
    drive(-50, 50, 10); //turn right at half speed
    heading=Gyro.rotation(deg);  //measure the heading of the robot
  }
  drive(0, 0, 0);  //stop the drive
}



void intakeStart(){
  intake.spin(reverse, 100, percent);
}

void intakeStop(){
  intake.stop();
}

void lift(int speed){
  lift11.spin(forward, speed, percent);
  lift55.spin(forward, speed, percent);
}

void liftStop(){
  lift11.stop();
  lift55.stop();
}

void clampOpen(){
  p1.set(true);
}

void clampClose(){
  p1.set(false);
}



float pi=3.142;
float dia=3.25;
float gearRatio = 4/3;

void armSet(float target){
  float x = 0;
  arm.setPosition(0,deg);
  x = arm.position(deg);

  if(target>=0){
    while (x <= target ) { 
    arm.spin(forward,100,percent);

      x=arm.position(deg); /*Brain.Screen.printAt(10, 20, "degrees = %0.2f", x);*/
  
    }
    }
  else if(target<=0){
    while (x>=target){
      arm.spin(reverse,100,percent);

      x=arm.position(deg); /*Brain.Screen.printAt(10, 20, "degrees = %0.2f", x);*/
    }
    
  }
  arm.stop();
}

void liftSet(float target){
  
  float x=0;
lift11.setPosition(0,deg);
x = lift11.position(deg)*pi*dia*gearRatio;

if (target >= 0 ){ //if your target is greater than 0 we will drive forward 
  while (x <= target ) { 
  lift(50);
  //use gyro here
  x = lift11.position(deg)*dia*pi*gearRatio; Brain.Screen.printAt(10, 20, "inches = %0.2f", x); 
  } 
} 
else if (target < 0){ 
    while (x >= target){ //target less than 0 the robot will drive backward
      lift(-50); 
      x = lift11.position(deg)*dia*pi*gearRatio; Brain.Screen.printAt(10, 20, "inches = %0.2f", x);
    }
} 

drive(0, 0, 0); 
}

void stakeMode(){
  int distance = Distance1.objectDistance(mm);
  float discs =0;
  lift11.setPosition(0,rev);
  
  armSet(15);

  intakeStart();
  //when distance<=110 && distance>=90 rotate 160degrees,discs++

  if (discs<=2 && distance<=110 && distance>=90){
    liftSet(-160);
    discs++;
  }
  else if(discs=2){
    armSet(30);
    discs=3;
  }
  armSet(30);
}

void inchDrive(float target) {
float x=0;
LFM.setPosition(0,rev);
x = LFM.position(rev)*pi*dia*gearRatio;

if (target >= 0 ){ //if your target is greater than 0 we will drive forward 
  while (x <= target ) { 
  drive(35, 35, 10);
  //use gyro here
  x = LFM.position(rev)*dia*pi*gearRatio; Brain.Screen.printAt(10, 20, "inches = %0.2f", x); 
  } 
} 
else if (target < 0){ 
    while (x >= target){ //target less than 0 the robot will drive backward
      drive(-35, -35, 10); 
      x = LFM.position(rev)*dia*pi*gearRatio; Brain.Screen.printAt(10, 20, "inches = %0.2f", x);
    }
} 
drive(0, 0, 0); 
}

/*void inchDrive(float target) {
  Brain.Screen.printAt(1, 20, "Inch Drive Start");
  float x = 0.0;
  float error = target - x;
  float speed = 75.0;
  float accuracy = 0.2;
  float ks = 1.0;
  float yaw = 0.0;
  float lspeed = speed * fabs(error) / error - ks * yaw;
  float rspeed = speed * fabs(error) / error + ks * yaw;
  
  Gyro.setRotation(0.0, deg);
  LBM.setRotation(0, rev);
  while (fabs(error) > accuracy) {
  drive(lspeed, rspeed, 10);
  x = LBM.position(rev) * Pi * D * G;
  error = target - x;
  yaw = Gyro.rotation(degrees);
  lspeed = speed * fabs(error) / error - ks * yaw;
  rspeed = speed * fabs(error) / error + ks * yaw;
  }
  driveBrake();
  }*/

void readIMU() {
 double heading20 = 0;
 double pitch20 = 0;
 double roll20 = 0;
 double yaw20 = 0;
 double rotation20 = 0;
 double ax = 0;
 double ay = 0; 
 double az = 0;

	heading20 = Gyro.heading(degrees);
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
	
	wait(20,msec);
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

void pre_auton(void) {

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...

  while(Gyro.isCalibrating()){     // Wait for Gyro Calibration , Sleep but Allow other tasks to run
      this_thread::sleep_for(20);
    }
  
    while (true){
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

void autonomous(void) {
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
    readIMU(); //print direction we are facing
    int left = Controller1.Axis3.position();
    int right = Controller1.Axis2.position();
    drive(left,right,10);

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
