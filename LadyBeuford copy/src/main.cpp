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

const int REQUIRED_PRESSES = 5;  // Number of presses required for password
int pressCount = 0;              // Counter for button presses
bool isPasswordCorrect = false;  // Flag to check if password is correct

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

void gyroTurn(float targetHeading, int timeout, double kp = 2)
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
    driveVolts(speed, -speed, 10); // turn right at speed
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

// Function to handle password input
void checkPassword() {
    // Check if Button A is pressed
    if (Controller.ButtonA.pressing()) {
        pressCount++; // Increment the press count
        wait(300, msec); // Debounce the button press (wait to avoid multiple counts for a single press)
    }

    // Check if the correct number of presses is entered
    if (pressCount >= REQUIRED_PRESSES) {
        isPasswordCorrect = true;
        pressCount = 0; // Reset press count after correct password is entered
        Brain.Screen.clearScreen();
        Brain.Screen.print("Password Correct!");
    }
    else if (pressCount > 0 && pressCount < REQUIRED_PRESSES) {
        // Provide feedback to the user on how many presses are left
        Brain.Screen.clearScreen();
        Brain.Screen.print("Password Incorrect");
    }
}

// User control function with password check
void usercontrol(void) {
    // Check password system
    checkPassword();
  
    // If password is not correct, lock the robot from moving
    if (!isPasswordCorrect) {
        Brain.Screen.clearScreen();
        Brain.Screen.print("Enter Correct Password");
        wait(20, msec);
        return; // Exit early to prevent any further control
    }
  
    // Once the password is correct, proceed with regular control
    Controller.ButtonX.pressed(mogo_mech_control);
    Controller.ButtonR1.pressed(nextState);
    Controller.ButtonR2.pressed(prevState);

    // Main control loop
    float sensitivity = 1.0;

    left_motor_front.setBrake(coast);
    left_motor_middle.setBrake(coast);
    left_motor_back.setBrake(coast);
    right_motor_front.setBrake(coast);
    right_motor_middle.setBrake(coast);
    right_motor_back.setBrake(coast);

    hook.setBrake(coast);
    LB.setBrake(hold);
    
    left_motor_front.setVelocity(100, pct);
    left_motor_middle.setVelocity(100, pct);
    left_motor_back.setVelocity(100, pct);
    right_motor_front.setVelocity(100, pct);
    right_motor_middle.setVelocity(100, pct);
    right_motor_back.setVelocity(100, pct);

    hook.setVelocity(100, pct);
    LB.setVelocity(100, pct);

    while (true) {
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
        if (Controller.ButtonL1.pressing()) {
            hook.spin(fwd, 100, pct);
        } else if (Controller.ButtonL2.pressing()) {
            hook.spin(fwd, -100, pct);
        } else {
            hook.stop();
        }

        // LB control
        if (Controller.ButtonR1.pressing()) {
            LB.spin(fwd, 100, pct);
        } else if (Controller.ButtonR2.pressing()) {
            LB.spin(fwd, -100, pct);
        } else {
            LB.stop();
        }

        // Wait to prevent resource wastage
        wait(20, msec);
    }
}

// Main function
int main() {
    // Set up callbacks for autonomous and driver control periods
    Competition.autonomous(autonomous);
    Competition.drivercontrol(usercontrol);

    // Run the pre-autonomous function
    pre_auton();

    // Wait a moment before starting
    wait(10, msec);

    // Main loop to keep the program alive
    while (true) {
        // Update the robot screen with status messages
        if (!isPasswordCorrect) {
            Brain.Screen.clearScreen();
            Brain.Screen.print("Enter Correct Password");
        }

        wait(100, msec); // Sleep to reduce CPU usage
    }
}
