/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Henry Pan                                                 */
/*    Created:      9/23/2024, 4:58:36 PM                                     */
/*    Description:  VEX High Stakes                                           */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;
brain Brain;
controller Controller;

// Left motors
motor left_motor1 = motor(PORT10, ratio6_1, true);
motor left_motor2 = motor(PORT9, ratio6_1, true);
motor left_motor3 = motor(PORT20, ratio6_1, false);

// Right motors
motor right_motor1 = motor(PORT1, ratio6_1, false);
motor right_motor2 = motor(PORT2, ratio6_1, false);
motor right_motor3 = motor(PORT14, ratio6_1, true);

motor conveyor = motor(PORT18, ratio6_1, true);
motor lift = motor(PORT15, ratio18_1, false);

digital_out mogo_mech = digital_out(Brain.ThreeWirePort.A);

inertial Inertial = inertial(PORT13);

// bool mogo_mech_bool = true;
bool verified = false;

// PID Variables
double previousErrorDistance = 0;
double integralDistance = 0;
double previousErrorHeading = 0;
double integralHeading = 0;
const double wheelDiameter = 2.75;                                   // Wheel diameter in inches
const double WHEELCIRCUMFERENCE = wheelDiameter * 3.141592653589793; // Circumference in inches
const double PI = 3.1415;
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
  left_motor1.spin(fwd, lspeed, voltageUnits::mV);
  left_motor2.spin(fwd, lspeed, voltageUnits::mV);
  left_motor3.spin(fwd, lspeed, voltageUnits::mV);
  right_motor1.spin(fwd, rspeed, voltageUnits::mV);
  right_motor2.spin(fwd, rspeed, voltageUnits::mV);
  right_motor3.spin(fwd, rspeed, voltageUnits::mV);
  wait(wt, msec);
}

void driveBrake()
{
  left_motor1.stop(brake);
  left_motor2.stop(brake);
  left_motor3.stop(brake);
  right_motor1.stop(brake);
  right_motor2.stop(brake);
  right_motor3.stop(brake);
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

// InchDrive helped by Jeffrey
void inchDrive(float targetDistanceInches, double targetVelocity, float timeout, float wait_ms = 10, float kp = 5.0)
{
  left_motor1.setPosition(0.0, rev); // resets rotations of left_motor1
  float actualDistance = 0.0;        // actual distance calculates rotations of left_motor1
  float error = targetDistanceInches - actualDistance;

  float accuracy = 0.5;
  timer t2;

  while (t2.time(msec) < timeout)
  {
    targetVelocity = kp * error;
    // Drives forward until actual distance meets target distance

    driveVolts(targetVelocity, targetVelocity, 100);

    actualDistance = left_motor1.position(rev) * PI * D * G;
    error = targetDistanceInches - actualDistance;
    Brain.Screen.print(error);
    Brain.Screen.newLine();
  }
  driveBrake();
  wait(wait_ms, msec);
}

void gyroTurn(float targetHeading, int timeout)
{

  float heading = 0.0; // initialize a variable for heading
  // Inertial.setRotation(0.0, degrees);  //reset Gyro to zero degrees
  Inertial.resetRotation();

  float error = targetHeading - heading;
  float accuracy = 2.0;
  double kp = 2;
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
  left_motor1.resetPosition();
  left_motor2.resetPosition();
  left_motor3.resetPosition();
  right_motor1.resetPosition();
  right_motor2.resetPosition();
  right_motor3.resetPosition();

  double actualDistance = 0;
  double errorDistance = 0;
  actualDistance = (left_motor1.position(deg));
  errorDistance = targetDistance - actualDistance;

  while (fabs(errorDistance) > 1)
  { // 1-degree tolerance
    double Speed = (KpD * errorDistance);

    left_motor1.spin(forward, Speed, pct);
    left_motor2.spin(forward, Speed, pct);
    left_motor3.spin(forward, Speed, pct);
    right_motor1.spin(forward, Speed, pct);
    right_motor2.spin(forward, Speed, pct);
    right_motor3.spin(forward, Speed, pct);

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
  left_motor1.stop();
  left_motor2.stop();
  left_motor3.stop();
  right_motor1.stop();
  right_motor2.stop();
  right_motor3.stop();
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
    left_motor1.spin(forward, -outputHeading, percent);
    left_motor2.spin(forward, -outputHeading, percent);
    left_motor3.spin(forward, -outputHeading, percent);
    right_motor1.spin(forward, outputHeading, percent);
    right_motor2.spin(forward, outputHeading, percent);
    right_motor3.spin(forward, outputHeading, percent);

    previousErrorHeading = errorHeading;
    this_thread::sleep_for(20); // Delay for stability
  }

  // Stop motors after the turn
  left_motor1.stop();
  left_motor2.stop();
  left_motor3.stop();
  right_motor1.stop();
  right_motor2.stop();
  right_motor3.stop();
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

  inchDrive(-31, 75, 850, 10);
  gyroTurn(31, 450);
  inchDrive(-19.5, 80, 500, 50);
  wait(100, msec);
  mogo_mech.set(true);
  conveyor.setMaxTorque(100, pct);
  conveyor.setVelocity(100, pct);
  conveyor.spin(forward);
  wait(500, msec);
  inchDrive(12, 80, 800, 10);
  gyroTurn(-30, 500);
  inchDrive(35, 80, 1100, 10);
  gyroTurn(35, 500);
  // conveyor.stop();
  inchDrive(30, 60, 200, 0);
  wait(500, msec);
  inchDrive(-10, 80, 200, 0);
  gyroTurn(180, 500);
  mogo_mech.set(false);
  

  

  /*
  gyroTurn(-180, 1100);
  inchDrive(-20, 70, 500, 10);
  mogo_mech.set(false);
  wait(10, msec);
  gyroTurn(15, 500);
  conveyor.spin(fwd);
  inchDrive(55, 80, 900, 50, 2);
  wait(500, msec);
  conveyor.stop();
  inchDrive(-10, 80, 500, 50, 2);
  gyroTurn(-101, 500);
  inchDrive(-22, 80, 700, 50);
  mogo_mech.set(true);
  conveyor.spin(fwd);
  wait(2, sec);
  conveyor.stop();
  gyroTurn(-30, 800);
  inchDrive(-17, 100, 500, 10);
  */

  // inchDrive(0, 80, 500, 10);
  // conveyor.stop();

  // inchDrive(-28, 80, 1200, 30);
  // gyroTurn(-30, 800);
  // inchDrive(-23, 500, 1200, 10, 4.3);
  // mogo_mech.set(true);
  // conveyor.spin(fwd, 100, pct);
  // wait(2, sec);
  // conveyor.stop();
  // gyroTurn(-60, 500);
  // inchDrive(28, 80, 1000, 10);
  // mogo_mech.set(false);
  // wait(0.5, sec);
  // inchDrive(-10, 80, 1200, 1000);

  // gyroTurn(135.0, 30);
  // inchDrive(34, 80, 100);
  // mogo_mech_control();
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
  // User control code here, inside the loop

  // This is the main execution loop for the user control program.
  // Each time through the loop your program should update motor + servo
  // values based on feedback from the joysticks.

  // ........................................................................
  float sensitivity = 1.0;

  left_motor1.setBrake(coast);
  left_motor2.setBrake(coast);
  left_motor3.setBrake(coast);
  right_motor1.setBrake(coast);
  right_motor2.setBrake(coast);
  right_motor3.setBrake(coast);

  conveyor.setBrake(coast);
  lift.setBrake(hold);

  left_motor1.setVelocity(100, pct);
  left_motor2.setVelocity(100, pct);
  left_motor3.setVelocity(100, pct);
  right_motor1.setVelocity(100, pct);
  right_motor2.setVelocity(100, pct);
  right_motor3.setVelocity(100, pct);

  conveyor.setVelocity(100, pct);
  // lift.setVelocity(100, pct);
  while (true)
  {
    
    double forward = Controller.Axis3.position();
    double turn = Controller.Axis1.position();

    // Calculate motor speeds
    double left_speed = (forward + turn) * sensitivity * 12 / 100;
    double right_speed = (forward - turn) * sensitivity * 12 / 100;

    // Set motor speeds
    left_motor1.spin(fwd, left_speed, volt);
    left_motor2.spin(fwd, left_speed, volt);
    left_motor3.spin(fwd, left_speed, volt);
    right_motor1.spin(fwd, right_speed, volt);
    right_motor2.spin(fwd, right_speed, volt);
    right_motor3.spin(fwd, right_speed, volt);

    // Intake Conveyor
    if (Controller.ButtonL1.pressing())
    {
      conveyor.spin(fwd, 100, pct);
    }
    else if (Controller.ButtonL2.pressing())
    {
      conveyor.spin(fwd, -100, pct);
    }
    else
    {
      conveyor.stop();
    }
    // Lift
    if (Controller.ButtonR1.pressing())
    {
      lift.spin(fwd, 100, pct);
    }
    else if (Controller.ButtonR2.pressing())
    {
      lift.spin(fwd, -100, pct);
    }
    else
    {

      lift.stop();
    }

    // ........................................................................

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.

    
  }

}

void brain_screen()
{
  Brain.Screen.setCursor(1, 1);

  if (left_motor1.installed())
  {
    Brain.Screen.setPenColor(white);
    Brain.Screen.print("Left Motor 1: ");
    Brain.Screen.print(left_motor1.temperature(percent));
    Brain.Screen.print("% Temperature");
    Brain.Screen.newLine();
  }
  else
  {
    Brain.Screen.setPenColor(red);
    Brain.Screen.print("Left Motor 1: PROBLEM DETECTED!");
    Brain.Screen.newLine();
  }
  /**/
  if (left_motor2.installed())
  {
    Brain.Screen.setPenColor(white);
    Brain.Screen.print("Left Motor 2: ");
    Brain.Screen.print(left_motor1.temperature(percent));
    Brain.Screen.print("% Temperature");
    Brain.Screen.newLine();
  }
  else
  {
    Brain.Screen.setPenColor(red);
    Brain.Screen.print("Left Motor 2: PROBLEM DETECTED!");
    Brain.Screen.newLine();
  }
  /**/
  if (left_motor3.installed())
  {
    Brain.Screen.setPenColor(white);
    Brain.Screen.print("Left Motor 3: ");
    Brain.Screen.print(left_motor1.temperature(percent));
    Brain.Screen.print("% Temperature");
    Brain.Screen.newLine();
  }
  else
  {
    Brain.Screen.setPenColor(red);
    Brain.Screen.print("Left Motor 3: PROBLEM DETECTED!");
    Brain.Screen.newLine();
  }
  /**/
  if (right_motor1.installed())
  {
    Brain.Screen.setPenColor(white);
    Brain.Screen.print("Right Motor 1: ");
    Brain.Screen.print(right_motor1.temperature(percent));
    Brain.Screen.print("% Temperature");
    Brain.Screen.newLine();
  }
  else
  {
    Brain.Screen.setPenColor(red);
    Brain.Screen.print("Right Motor 1: PROBLEM DETECTED!");
    Brain.Screen.newLine();
  }
  /**/
  if (right_motor2.installed())
  {
    Brain.Screen.setPenColor(white);
    Brain.Screen.print("Right Motor 2: ");
    Brain.Screen.print(right_motor2.temperature(percent));
    Brain.Screen.print("% Temperature");
    Brain.Screen.newLine();
  }
  else
  {
    Brain.Screen.setPenColor(red);
    Brain.Screen.print("Right Motor 2: PROBLEM DETECTED!");
    Brain.Screen.newLine();
  }
  /**/
  if (right_motor3.installed())
  {
    Brain.Screen.setPenColor(white);
    Brain.Screen.print("Right Motor 3: ");
    Brain.Screen.print(right_motor3.temperature(percent));
    Brain.Screen.print("% Temperature");
    Brain.Screen.newLine();
  }
  else
  {
    Brain.Screen.setPenColor(red);
    Brain.Screen.print("Right Motor 3: PROBLEM DETECTED!");
    Brain.Screen.newLine();
  }
  /**/
  if (conveyor.installed())
  {
    Brain.Screen.setPenColor(white);
    Brain.Screen.print("Conveyor: ");
    Brain.Screen.print(conveyor.temperature(percent));
    Brain.Screen.print("% Temperature");
    Brain.Screen.newLine();
  }
  else
  {
    Brain.Screen.setPenColor(red);
    Brain.Screen.print("Conveyor: PROBLEM DETECTED!");
    Brain.Screen.newLine();
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

  // Prevent main from exiting with an infinite loop.
  while (true)
  {
   
    brain_screen();
    // //checkPassword();
    // if (verified == true) {
    //   usercontrol();
    // }

    wait(100, msec);
  }
}
