/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Henry Pan                                                 */
/*    Created:      9/23/2024, 4:58:36 PM                                     */
/*    Description:  VEX High Stakes                                           */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// #include "robot.hpp"
#include "robot.h"
#include "brainDisplay.hpp"
#include "lift.hpp"

// bool mogo_mech_bool = true;
bool verified = false;

// PID Variables
double previousErrorDistance = 0;
double integralDistance = 0;
double previousErrorHeading = 0;
double integralHeading = 0;
const double wheelDiameter = 2.75;                                   // Wheel diameter in inches
const double WHEELCIRCUMFERENCE = wheelDiameter * 3.141592653589793; // Circumference in inches
const double PI = 3.1415265;
const double D = 2.75;
const double G = 3.0 / 4.0;

void mogo_mech_control()
{

  mogo_mech.set(!mogo_mech.value());
}


void driveVolts(int lspeed, int rspeed, int wt)
{
  lspeed = lspeed * 120;
  rspeed = rspeed * 120;
  // Set motor speeds
  left_motor_front.spin(fwd, lspeed, voltageUnits::mV);
  left_motor_middle.spin(fwd, lspeed, voltageUnits::mV);
  left_motor_back.spin(fwd, lspeed, voltageUnits::mV);
  right_motor_front.spin(fwd, rspeed, voltageUnits::mV);
  right_motor_middle.spin(fwd, rspeed, voltageUnits::mV);
  right_motor_back.spin(fwd, rspeed, voltageUnits::mV);
  wait(wt, msec);
}

void driveBrake()
{
  left_motor_front.stop(brake);
  left_motor_middle.stop(brake);
  left_motor_back.stop(brake);
  right_motor_front.stop(brake);
  right_motor_middle.stop(brake);
  right_motor_back.stop(brake);
}

void clamp(double &value, double minValue, double maxValue)
{
  if (value > maxValue)
  {
    value = maxValue;
  }
  else if (value < minValue)
  {
    value = minValue;
  }
}

void resetPID()
{
  previousErrorDistance = 0;
  integralDistance = 0;
}


void inchDrive(float targetDistanceInches, double targetVelocity, float timeout, float wait_ms = 10, float kp = 5.0)
{
  left_motor_front.setPosition(0.0, rev); // resets rotations of left_motor_front
  float actualDistance = 0.0;        // actual distance calculates rotations of left_motor_front
  float error = targetDistanceInches - actualDistance;

  float accuracy = 0.5;
  timer t2;

  while (t2.time(msec) < timeout)
  {
    targetVelocity = kp * error;
    // Drives forward until actual distance meets target distance

    driveVolts(targetVelocity, targetVelocity, 100);

    actualDistance = left_motor_front.position(rev) * PI * D * G;
    error = targetDistanceInches - actualDistance;
    Brain.Screen.print(error);
    Brain.Screen.newLine();
  }
  driveBrake();
  wait(wait_ms, msec);
}

void gyroTurn(float targetHeading, int timeout, double kp=2)
{

  float heading = 0.0; // initialize a variable for heading
  // Inertial.setRotation(0.0, degrees);  //reset Gyro to zero degrees
  Inertial.resetRotation();

  float error = targetHeading - heading;
  float accuracy = 2.0;
  
  timer t1;

  while (t1.time(msec) < timeout)
  {
    heading = Inertial.rotation(deg); // measure the heading of the robot
    error = targetHeading - heading;
    float speed = error * kp;
    // if(error>0){
    driveVolts(speed, -speed, 10); // turn right at speed
    // }
    // else{
    //   driveVolts(speed, -speed, 10); //turn left at speed
    // }
    // wait(20,msec);
  }
  driveBrake(); // stop the drive
}

void drivePID(double targetDistanceInInches, double targetVelocity, double KpD, double KiD, double KdD)
{
  resetPID();

  // Convert target distance from inches to degrees
  double targetDistance = (targetDistanceInInches / WHEELCIRCUMFERENCE) * 360; // Degrees

  // Reset motor positions
  left_motor_front.resetPosition();
  left_motor_middle.resetPosition();
  left_motor_back.resetPosition();
  right_motor_front.resetPosition();
  right_motor_middle.resetPosition();
  right_motor_back.resetPosition();

  double actualDistance = 0;
  double errorDistance = 0;
  actualDistance = (left_motor_front.position(deg));
  errorDistance = targetDistance - actualDistance;

  while (fabs(errorDistance) > 1)
  { // 1-degree tolerance
    double Speed = (KpD * errorDistance);

    left_motor_front.spin(forward, Speed, pct);
    left_motor_middle.spin(forward, Speed, pct);
    left_motor_back.spin(forward, Speed, pct);
    right_motor_front.spin(forward, Speed, pct);
    right_motor_middle.spin(forward, Speed, pct);
    right_motor_back.spin(forward, Speed, pct);

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print(errorDistance);
    Brain.Screen.newLine();
    Brain.Screen.print(actualDistance);
    Brain.Screen.newLine();
    Brain.Screen.print(Speed);
    wait(20, msec); // Small delay for stability
  }

  // Stop motors after the movement
  left_motor_front.stop();
  left_motor_middle.stop();
  left_motor_back.stop();
  right_motor_front.stop();
  right_motor_middle.stop();
  right_motor_back.stop();
}

void turnToHeading(double targetHeading, double KpH, double KiH, double KdH)
{
  // Reset Inertial
  Inertial.resetRotation();
  resetPID(); // Reset PID values for heading control

  while (fabs(targetHeading - Inertial.heading(deg)) > 2)
  { // 2-degree tolerance
    double actualHeading = Inertial.heading(deg);

    double errorHeading = targetHeading - actualHeading;
    integralHeading += errorHeading;

    // Prevent integral windup
    if (fabs(errorHeading) < 10)
    {
      integralHeading += errorHeading;
    }

    double derivativeHeading = errorHeading - previousErrorHeading;
    double outputHeading = KpH * errorHeading + KiH * integralHeading + KdH * derivativeHeading;

    // Clamp output to motor limits
    clamp(outputHeading, -100.0, 100.0);

    // Set motor speeds for turning
    left_motor_front.spin(forward, -outputHeading, percent);
    left_motor_middle.spin(forward, -outputHeading, percent);
    left_motor_back.spin(forward, -outputHeading, percent);
    right_motor_front.spin(forward, outputHeading, percent);
    right_motor_middle.spin(forward, outputHeading, percent);
    right_motor_back.spin(forward, outputHeading, percent);

    previousErrorHeading = errorHeading;
    this_thread::sleep_for(20); // Delay for stability
  }

  // Stop motors after the turn
  left_motor_front.stop();
  left_motor_middle.stop();
  left_motor_back.stop();
  right_motor_front.stop();
  right_motor_middle.stop();
  right_motor_back.stop();
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
  hook.setMaxTorque(110, pct);
  hook.setVelocity(100, pct);
  Inertial.calibrate();
  while (Inertial.isCalibrating())
  {
    wait(10, msec);
  }

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                          */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void)
{
  Controller.Screen.newLine();
  Controller.Screen.print("Auton in Progress");
  inchDrive(-26, 80, 900, 0);
  gyroTurn(-29, 650);
  inchDrive(-25, 80, 1000, 0, 3);
  //wait(50, msec);
  mogo_mech.set(true);
  wait(50, msec);
  
  hook.spin(forward);
  wait(500, msec);
  //change
  //clamped
  inchDrive(16, 80, 1000, 0);
  gyroTurn(30, 700);
  //changed 10
  inchDrive(33, 10, 1100, 0);
  gyroTurn(-37, 800);
  // hook.stop();
  inchDrive(15, 100, 1100, 0, 2);
  
  inchDrive(-12, 80, 700, 0);
  wait(500, msec);
  gyroTurn(190, 900, 1);
  mogo_mech.set(false);

  //trying to change speed here.
  inchDrive(27, 10, 1100, 30);
  hook.stop();
  gyroTurn(105, 900);
  inchDrive(-18, 80, 900, 0);
  mogo_mech.set(true);
  hook.spin(forward);
  inchDrive(-6, 80, 500, 0);
  gyroTurn(45, 500);
  inchDrive(-17, 80, 800, 0);

  //After corner
  // inchDrive(-2, 80, 850, 0);
  // gyroTurn(-50, 1000);
  // mogo_mech.set(false);
  // hook.stop();
  //droped mogo
  //inchDrive(-25, 750000000, 850, 10);
  //mogo_mech.set(true);

  //gyroTurn(20, 200);
  //change velocity on 303,305
  //3rd 5 to 200
  //inchDrive(5, 5, 200, 10);
  //gyroTurn(-90, 200);
  //inchDrive(10, 60, 500, 10);
  //wait(500, msec);
  //added 12:20
  //inchDrive(-5, 800, 500, 10);
  //gyroTurn(-700, 200);
  
  // inchDrive(-10, 80, 200, 0);
  // gyroTurn(180, 500);
  // mogo_mech.set(false);
  Controller.Screen.clearLine();
  
}
// ..........................................................................
// Wait a moment
// ..........................................................................

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
  
  Controller.ButtonX.pressed(mogo_mech_control);
  Controller.ButtonR1.pressed(nextState);
  Controller.ButtonR2.pressed(prevState);
  // User control code here, inside the loop

  // This is the main execution loop for the user control program.
  // Each time through the loop your program should update motor + servo
  // values based on feedback from the joysticks.

  // ........................................................................
  float sensitivity = 1.0;

  left_motor_front.setBrake(coast);
  left_motor_middle.setBrake(coast);
  left_motor_back.setBrake(coast);
  right_motor_front.setBrake(coast);
  right_motor_middle.setBrake(coast);
  right_motor_back.setBrake(coast);

  hook.setBrake(coast);
  LB.setBrake(coast);
  
  

  left_motor_front.setVelocity(100, pct);
  left_motor_middle.setVelocity(100, pct);
  left_motor_back.setVelocity(100, pct);
  right_motor_front.setVelocity(100, pct);
  right_motor_middle.setVelocity(100, pct);
  right_motor_back.setVelocity(100, pct);

  hook.setVelocity(100, pct);
  LB.setVelocity(100, pct);
  // lift.setVelocity(100, pct);
  while (true)
  {
    
    double forward = Controller.Axis3.position();
    double turn = Controller.Axis1.position();

    // Calculate motor speeds
    double left_speed = (forward + turn) * sensitivity * 12 / 100;
    double right_speed = (forward - turn) * sensitivity * 12 / 100;

    // Set motor speeds
    left_motor_front.spin(fwd, left_speed, volt);
    left_motor_middle.spin(fwd, left_speed, volt);
    left_motor_back.spin(fwd, left_speed, volt);
    right_motor_front.spin(fwd, right_speed, volt);
    right_motor_middle.spin(fwd, right_speed, volt);
    right_motor_back.spin(fwd, right_speed, volt);

    // Intake hook
    if (Controller.ButtonL1.pressing())
    {
      hook.spin(fwd, 100, pct);
    }
    else if (Controller.ButtonL2.pressing())
    {
      hook.spin(fwd, -100, pct);
    }
    else
    {
      hook.stop();
    }

    liftControl();

    // ........................................................................

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.

    
  }

}



int main()
{
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autono mous function.
  pre_auton();



  wait(10, msec);
  drawLogo();

  // Prevent main from exiting with an infinite loop.
  while (true)
  {
    int idle = 1;
    brain_screen();
    // //checkPassword();
    // if (verified == true) {
    //   usercontrol();
    // }

    wait(100, msec);
  }
}
