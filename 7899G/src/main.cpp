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
motor right_motor3 = motor(PORT11, ratio6_1, true);

motor conveyor = motor(PORT18, ratio6_1, true);
motor lift = motor(PORT15, ratio18_1, false);

digital_out mogo_mech = digital_out(Brain.ThreeWirePort.A);

inertial Inertial = inertial(PORT12);

bool mogo_mech_bool = true;
bool verified = false;

// PID Variables
double previousErrorDistance = 0;
double integralDistance = 0;
double previousErrorHeading = 0;
double integralHeading = 0;
const double wheelDiameter = 2.75; // Wheel diameter in inches
const double wheelCircumference = wheelDiameter * 3.141592653589793; // Circumference in inches


void mogo_mech_control() {
  mogo_mech_bool = !mogo_mech_bool;
  mogo_mech.set(mogo_mech_bool);
}

//code by Will
/**
void driveforward(double inches, double velocity, bool wait) {
  
  double rotation = (360.0 / wheelCircumference) * inches; 

  left_motor1.setBrake(coast);
  left_motor2.setBrake(coast);
  left_motor3.setBrake(coast);
  right_motor1.setBrake(coast);
  right_motor2.setBrake(coast);
  right_motor3.setBrake(coast);

  left_motor1.resetPosition();
  left_motor2.resetPosition();
  left_motor3.resetPosition();
  right_motor1.resetPosition();
  right_motor2.resetPosition();
  right_motor3.resetPosition();

  left_motor1.setVelocity(velocity, pct);
  left_motor2.setVelocity(velocity, pct);
  left_motor3.setVelocity(velocity, pct);
  right_motor1.setVelocity(velocity, pct);
  right_motor2.setVelocity(velocity, pct);
  right_motor3.setVelocity(velocity, pct);



  left_motor1.spinFor(rotation, degrees, false);
  left_motor2.spinFor(rotation, degrees, false);
  left_motor3.spinFor(rotation, degrees, false);
  right_motor1.spinFor(rotation, degrees, false);
  right_motor2.spinFor(rotation, degrees, false);
  right_motor3.spinFor(rotation, degrees, wait);


  Brain.Screen.print(rotation);
}
**/


void clamp(double &value, double minValue, double maxValue) {
    if (value > maxValue) {
        value = maxValue;
    } else if (value < minValue) {
        value = minValue;
    }
}

void resetPID() {
    previousErrorDistance = 0;
    integralDistance = 0;
}

//InchDrive helped by Jeffrey
void inchDrive(float targetDistanceInches, double targetVelocity){
  float targetDistanceInDegrees = (targetDistanceInches/wheelCircumference) * 360; //converts distance to degrees
  left_motor1.resetPosition(); //resets rotations of left_motor1
  float actualDistance = left_motor1.position(deg); //actual distance calculates rotations of left_motor1
  Brain.Screen.print(targetDistanceInDegrees);
  Brain.Screen.newLine();
  while(actualDistance < targetDistanceInDegrees){ //Drives forward until actual distance meets target distance
    left_motor1.spin(forward, targetVelocity, pct);
    left_motor2.spin(forward, targetVelocity, pct);
    left_motor3.spin(forward, targetVelocity, pct);
    right_motor1.spin(forward, targetVelocity, pct);
    right_motor2.spin(forward, targetVelocity, pct);
    right_motor3.spin(forward, targetVelocity, pct);
    actualDistance = left_motor1.position(deg);
    Brain.Screen.print(actualDistance);
    Brain.Screen.newLine();
  }\
  left_motor1.stop();
  left_motor2.stop();
  left_motor3.stop();
  right_motor1.stop();
  right_motor2.stop();
  right_motor3.stop();
} 

void drivePID(double targetDistanceInInches, double targetVelocity, double KpD, double KiD, double KdD) {
    resetPID();

    // Convert target distance from inches to degrees
    double targetDistance = (targetDistanceInInches / wheelCircumference) * 360; // Degrees

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

    while (fabs(errorDistance) > 1) { // 1-degree tolerance
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

void turnToHeading(double targetHeading, double KpH, double KiH, double KdH) {
    // Reset Inertial
    Inertial.resetRotation();
    resetPID(); // Reset PID values for heading control

    while (fabs(targetHeading - Inertial.heading(deg)) > 2) { // 2-degree tolerance
        double actualHeading = Inertial.heading(deg);

        double errorHeading = targetHeading - actualHeading;
        integralHeading += errorHeading;

        // Prevent integral windup
        if (fabs(errorHeading) < 10) {
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


void autonomous(void){
      inchDrive(24,50);  
      turnToHeading(90, 0.1, 0.1, 0.1); //might need to change the numbers


      wait(10,msec); 
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


void usercontrol(void) {
  
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
  lift.setVelocity(100, pct);
  while (true) {
  double forward = Controller.Axis3.position();
  double turn = Controller.Axis1.position();

  // Calculate motor speeds
  double left_speed = (forward + turn) * sensitivity * 12/100;
  double right_speed = (forward - turn) * sensitivity * 12/100;

  // Set motor speeds
  left_motor1.spin(fwd, left_speed, volt);
  left_motor2.spin(fwd, left_speed, volt);
  left_motor3.spin(fwd, left_speed, volt);
  right_motor1.spin(fwd, right_speed, volt);
  right_motor2.spin(fwd, right_speed, volt);
  right_motor3.spin(fwd, right_speed, volt);

  // Intake Conveyor
  if (Controller.ButtonL1.pressing()) {
    conveyor.spin(fwd, 100, pct);
  }
  else if (Controller.ButtonL2.pressing()) {
    conveyor.spin(fwd, -100, pct);
  }
  else {
    conveyor.stop();
    
  }
  // Lift
  if (Controller.ButtonR1.pressing()) {
    lift.spin(fwd, 100, pct);
  }
  else if (Controller.ButtonR2.pressing()) {
    lift.spin(fwd, -100, pct);
  }
  else {

      lift.stop();

  }

  // ........................................................................

  wait(20, msec); // Sleep the task for a short amount of time to
                  // prevent wasted resources.
  }
  
}


void brain_screen() {
  Brain.Screen.setCursor(1, 1);

  Brain.Screen.print("Left Motor 1: ");
  Brain.Screen.print(left_motor1.temperature(percent));
  Brain.Screen.print("% Temperature");
  Brain.Screen.newLine();

  Brain.Screen.print("Left Motor 2: ");
  Brain.Screen.print(left_motor2.temperature(percent));
  Brain.Screen.print("% Temperature");
  Brain.Screen.newLine();

  Brain.Screen.print("Left Motor 3: ");
  Brain.Screen.print(left_motor3.temperature(percent));
  Brain.Screen.print("% Temperature");
  Brain.Screen.newLine();

  Brain.Screen.print("Right Motor 1: ");
  Brain.Screen.print(right_motor1.temperature(percent));
  Brain.Screen.print("% Temperature");
  Brain.Screen.newLine();

  Brain.Screen.print("Right Motor 2: ");
  Brain.Screen.print(right_motor2.temperature(percent));
  Brain.Screen.print("% Temperature");
  Brain.Screen.newLine();
  
  Brain.Screen.print("Right Motor 3: ");
  Brain.Screen.print(right_motor3.temperature(percent));
  Brain.Screen.print("% Temperature");
  Brain.Screen.newLine();

  
}

int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autono mous function.
  pre_auton();

  
  


  // Prevent main from exiting with an infinite loop.
  while (true) {
    

    brain_screen();
    // //checkPassword();
    // if (verified == true) {
    //   usercontrol();
    // }
    

    wait(100, msec);
  }
}
