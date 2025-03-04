
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

#include "lift.hpp"
#include "brainDisplay.hpp"
#include <iostream>

const double PI = 3.1415265;
const double D = 2.75;
const double G = 3.0 / 4.0;
const float W = 14.0; // width of rovot track

static bool verified = false;

bool isAutonomousRunning = false;

void setup()
{
  // if (Controller.ButtonY.pressing()) {
  //   step1 = true;
  // }
  // if (Controller.ButtonLeft.pressing()) {
  //   if (step1 == true) {
  //     step2 = true;
  //   }
  // }
  // if (Controller.ButtonRight.pressing()) {
  //   if (step2 == true) {
  //     verified = true;
  //   }
  // }

  static bool step1_complete = false;
  static bool step2_complete = false;

  // Check if ButtonY is pressed for step1
  if (Controller.ButtonY.pressing() && !step1_complete)
  {
    step1_complete = true;
  }

  // Check if ButtonLeft is pressed for step2, but only if step1 is complete
  if (Controller.ButtonLeft.pressing() && step1_complete && !step2_complete)
  {
    step2_complete = true;
  }

  // Check if ButtonRight is pressed to verify, but only if step2 is complete
  if (Controller.ButtonRight.pressing() && step2_complete && !verified)
  {
    verified = true;
  }
}

void mogo_mech_control()
{

  mogo_mech.set(!mogo_mech.value());
}

void hitler_mech_control()
{

  hitler_mech.set(!hitler_mech.value());
}

void moveLift()
{
  while (isAutonomousRunning)
  {
    // Add the logic for controlling the lift here
    // For example:
    liftControl();

    // Small delay to prevent busy-waiting and reduce CPU usage
    wait(20, msec);
  }
  // Perform cleanup if necessary
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

void arcTurn(float rd, float angle, float maxSpeed = 100)
{
  float kp = 1.0;
  float kd = 1.0;
  float targetArcLength = rd * 2 * PI * angle / 360.0;
  float arcLength = 0.0;
  float error = targetArcLength - arcLength;
  float oldError = error;
  float lspeed = maxSpeed * angle / fabs(angle);
  float rspeed = lspeed * (rd - W) * rd;
  float accuracy = 0.2;
  left_motor_front.setPosition(0.0, rev);
  left_motor_middle.setPosition(0.0, rev);
  left_motor_back.setPosition(0.0, rev);
  right_motor_front.setPosition(0.0, rev);
  right_motor_middle.setPosition(0.0, rev);
  right_motor_back.setPosition(0.0, rev);
  while (fabs(error) >= accuracy)
  {
    driveVolts(lspeed, rspeed, 10);
    arcLength = left_motor_middle.position(rev) * G * PI * D;
    oldError = error;
    error + targetArcLength - arcLength;
    lspeed = kp * error + kd + (error - oldError);
    if (fabs(lspeed) >= maxSpeed)
      lspeed = maxSpeed * error / fabs(error);
    rspeed = lspeed * (rd - W) / rd;
  }
  driveBrake();
  // working on
}

void driveAngle(float targetDistanceInches, float angle, float timeout, float wait_ms = 10, float kp = 6.0, float slew = -1, double maxVel = -1)
{
  left_motor_front.setPosition(0.0, rev); // resets rotations of left_motor_front
  float actualDistance = 0.0;             // actual distance calculates rotations of left_motor_front
  float error = targetDistanceInches - actualDistance;
  float targetVelocity;
  float accuracy = 0.5;
  float prevVelocity = 0;
  float turnKp = 0.8;
  timer t2;

  while (fabs(error) > accuracy)
  {
    float heading = Inertial.rotation(deg);
    float angularError = angle - heading;


    if (error > 180)
    {
      error = error - 360;
    }
    if (error < -180)
    {
      error = error + 360;
    }

    float angularVelocity = angularError * turnKp;

    targetVelocity = kp * error;
    int sign = targetVelocity > 0 ? 1 : - 1;
    if ((fabs(targetVelocity) + fabs(angularVelocity)) >= 100.0) {
      targetVelocity = sign * (100.0 - fabs(angularVelocity));
    }
    std::cout << targetVelocity << std::endl;
    if (slew != -1)
    {
      targetVelocity = sign * std::min(fabs(targetVelocity), fabs(prevVelocity)+ slew);
    }
    if (maxVel != -1) targetVelocity = sign * std::min(fabs(targetVelocity), maxVel);
    prevVelocity = targetVelocity;
    // Drives forward until actual distance meets target distance

    driveVolts(targetVelocity + angularVelocity, targetVelocity - angularVelocity, 10);

    actualDistance = left_motor_front.position(rev) * PI * D * G;
    error = targetDistanceInches - actualDistance;
    Brain.Screen.print(error);
    Brain.Screen.newLine();
    if (t2.time(msec) > timeout)
    {
      break;
    }
  }
  driveBrake();
  wait(wait_ms, msec);
}

void inchDrive(float targetDistanceInches, float timeout, float wait_ms = 10, float kp = 6.0, float slew = -1, double maxVel = -1)
{
  left_motor_front.setPosition(0.0, rev); // resets rotations of left_motor_front
  float actualDistance = 0.0;             // actual distance calculates rotations of left_motor_front
  float error = targetDistanceInches - actualDistance;
  float targetVelocity;
  float accuracy = 0.5;
  float prevVelocity = 0;
  timer t2;

  while (fabs(error) > accuracy)
  {
    targetVelocity = kp * error;
    int sign = targetVelocity > 0 ? 1: -1;
    if (slew != -1)
    {
      targetVelocity = sign * std::min(fabs(targetVelocity), fabs(prevVelocity)+ slew);
    }
    if (maxVel != -1) targetVelocity = sign * std::min(fabs(targetVelocity), maxVel);
    prevVelocity = targetVelocity;
    // Drives forward until actual distance meets target distance

    driveVolts(targetVelocity, targetVelocity, 10);

    actualDistance = left_motor_front.position(rev) * PI * D * G;
    error = targetDistanceInches - actualDistance;
    Brain.Screen.print(error);
    Brain.Screen.newLine();
    if (t2.time(msec) > timeout)
    {
      break;
    }
  }
  driveBrake();
  wait(wait_ms, msec);
}

// 99

void gyroTurn(float targetHeading, int timeout, double kp = 1.5)
{

  float heading = 0.0; // initialize a variable for heading
  // Inertial.setRotation(0.0, degrees);  //reset Gyro to zero degrees
  // Inertial.resetRotation();
  // 0

  float error = targetHeading - heading;
  float olderror = error;
  float accuracy = 0.5;
  int count = 0;
  float kd = 0.1;
  timer t1;

  while (fabs(error) > accuracy or count < 25)
  {
    float speed = error * kp + kd * (error - olderror);
    driveVolts(speed, -speed, 10);
    heading = Inertial.rotation(deg);
    std::cout << heading << "\n";
    // if (heading > 360)
    // {
    //   heading = heading - 360;
    // }
    // if (heading < -360)
    // {
    //   heading = heading + 360;
    // }
    // measure the heading of the robot
    error = targetHeading - heading;

    if (error > 180)
    {
      error = error - 360;
    }
    if (error < -180)
    {
      error = error + 360;
    }

    olderror = error;
    if (fabs(error) < accuracy)
    {
      count++;
    }
    else
    {
      count = 0;
    }

    if (t1.time(msec) > timeout)
    {
      break;
    }
  }
  std::cout << "Next Turn: " << "\n";
  driveBrake(); // stop the drive
  wait(50, msec);
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
  double pos = LBRotation.position(degrees);
  if (pos > 300) {
    LBRotation.setPosition(pos-360, degrees);
  }
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
  
  float startHeading = 0.0;
  isAutonomousRunning = true;
  thread liftThread = thread(moveLift);
  Inertial.setRotation(startHeading, degrees);

    // driveAngle(-95, 90, 4000);
// wait(1000000000000, sec);
  // inchDrive(24, 80, 1000);
  currentState = alliance;
  wait(800, msec);
  inchDrive(-9, 1000, 100);
  gyroTurn(-90, 1000);
  inchDrive(-20, 2000, 100, 5);
  mogo_mech.set(true);
  // gyroTurn(-180,2000);
  currentState = loading;
  hook.spin(fwd, 100, pct);
  // mogo clamped
  wait(200, msec);
  gyroTurn(-180, 1000);
  inchDrive(20, 1000, 100);
  // ring 1
  gyroTurn(-203, 1000);
  hook.stop();
  inchDrive(34.5, 3000);
  hook.spinFor(-100, deg);
  gyroTurn(-270, 1000);
  currentState = loading2;
  hook.spin(fwd, 100, pct);
  inchDrive(22, 1500, 10);
  currentState = scoring; // score 0 deg  - wall stake
  wait(500, msec);
  inchDrive(-11.5, 2000, 10);
  gyroTurn(0, 2000, 1.9);
  inchDrive(44, 3000, 10, 5);
  wait(250, msec);
  inchDrive(20, 1500, 10, 8);
  inchDrive(-10, 1500, 10, 10);
  gyroTurn(-270, 1500);
  inchDrive(15, 1500, 10, 5);
  wait(250, msec);
  inchDrive(-14, 2000, 8);
  gyroTurn(-140, 1500);
  mogo_mech.set(false);//first mogo drop 
  currentState = idle;
  inchDrive(-20, 2000, 8);
  inchDrive(13, 2000, 8);
  gyroTurn(90, 1500);
  inchDrive(28, 2000, 10, 4, -1, 40);
  // Inertial.setRotation(90,degrees);//wall reset
  wait(500, msec);
  driveAngle(-95, 90, 4000, 2.5, 2, 1);
  mogo_mech.set(true);
  //2nd part
  currentState = loading;
  wait(500, msec);
  gyroTurn(180, 1000);
  inchDrive(20, 1000, 100);
  gyroTurn(210, 1000);
  hook.stop();
  inchDrive(30.5, 3000);
  hook.spinFor(-100, deg);
  gyroTurn(270, 1000);
  currentState = loading2;
  hook.spin(fwd, 100, pct);
  inchDrive(23, 1500, 10);
  currentState = scoring; // score 0 deg  - wall stake
  wait(500, msec);
  inchDrive(-15, 2000, 10);
  gyroTurn(-0, 2000);
  inchDrive(44, 3000, 10, 5);
  wait(250, msec);
  inchDrive(18, 1500, 10, 8);
  inchDrive(-18, 1500, 10, 8);
  gyroTurn(-195, 2000);
  mogo_mech.set(false);
  inchDrive(-18, 1500, 10, 8);
  inchDrive(100, 3000, 2.5, 2, 1);
  gyroTurn(-100, 2000);
  inchDrive(14, 1500, 10, 8);

  isAutonomousRunning = false;
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

  Controller.ButtonB.pressed(mogo_mech_control);
  Controller.ButtonUp.pressed(hitler_mech_control);
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

  // hook.setBrake(coast);
  LB.setBrake(hold);

  left_motor_front.setVelocity(100, pct);
  left_motor_middle.setVelocity(100, pct);
  left_motor_back.setVelocity(100, pct);
  right_motor_front.setVelocity(100, pct);
  right_motor_middle.setVelocity(100, pct);
  right_motor_back.setVelocity(100, pct);

  hook.setVelocity(100, pct);
  LB.setVelocity(100, pct);

  while (true)
  {
    setup();
    verified = true;
    if (verified == true)
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

      // LB
      liftControl();

      // ........................................................................

      wait(20, msec); // Sleep the task for a short amount of time to
                      // prevent wasted resources.
    }
  }
}

int main()
{
  // Set up callbacks for autonomous and driver control periods.

  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();
  wait(10, msec);
  
  drawLogo();
  // Prevent main from exiting with an infinite loop.
  while (true)
  {
    brain_screen();
    int idle = 2;

    wait(100, msec);
  }
}
