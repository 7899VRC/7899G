/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       student                                                   */
/*    Created:      9/23/2024, 4:58:36 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;
brain Brain;
controller Controller;

// Left motors
motor left_motor1 = motor(PORT20, ratio6_1, false);
motor left_motor2 = motor(PORT19, ratio6_1, true);
motor left_motor3 = motor(PORT10, ratio6_1, true);

// Right motors
motor right_motor1 = motor(PORT11, ratio6_1, true);
motor right_motor2 = motor(PORT1, ratio6_1, false);
motor right_motor3 = motor(PORT12, ratio6_1, false);

motor conveyor = motor(PORT18, ratio6_1, true);
motor lift = motor(PORT5, ratio18_1, true);

digital_out mogo_mech = digital_out(Brain.ThreeWirePort.A);

bool mogo_mech_bool = true;

void mogo_mech_control() {
  mogo_mech_bool = !mogo_mech_bool;
  mogo_mech.set(mogo_mech_bool);
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
  Controller.ButtonL1.pressed(mogo_mech_control);

  // User control code here, inside the loop
  while (1) {
    


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
    if (Controller.ButtonB.pressing()) {
      lift.spin(fwd, 100, pct);
    }
    else if (Controller.ButtonX.pressing()) {
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
    usercontrol();
    wait(100, msec);
  }
}
