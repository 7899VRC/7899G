/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       danie                                                     */
/*    Created:      9/20/2024, 2:08:24 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
brain Brain;

controller  Controller1;

motor LF=motor(PORT1,ratio6_1,true);

motor LM=motor(PORT1,ratio6_1,true);

motor LB=motor(PORT2,ratio6_1,true);

motor RF=motor(PORT3,ratio6_1,false);

motor RM=motor(PORT3,ratio6_1,false);

motor RB=motor(PORT4,ratio6_1,false);

inertial Gyro=inertial(PORT18);

float D = 2.75;
float Pi = 3.14;
float G = 36.0/48.0;

  void drivevolts(int lspeed, int rspeed, int wt){
    lspeed=lspeed*120;
    rspeed=rspeed*120;
LF.spin(forward,lspeed,voltageUnits::mV);

LM.spin(forward,lspeed,voltageUnits::mV);

LB.spin(forward,lspeed,voltageUnits::mV);

RF.spin(forward,rspeed,voltageUnits::mV);

RM.spin(forward,rspeed,voltageUnits::mV);

RB.spin(forward,rspeed,voltageUnits::mV);

wait(wt,msec);
}



void drive(int lspeed, int rspeed, int wt){

LF.spin(forward,lspeed,percent);

LM.spin(forward,lspeed,percent);

LB.spin(forward,lspeed,percent);

RF.spin(forward,rspeed,percent);

RM.spin(forward,rspeed,percent);

RB.spin(forward,rspeed,percent);

wait(wt,msec);
}


void driveBrake(){
  LF.stop(brake);
  LM.stop(brake);
  LB.stop(brake);
  RF.stop(brake);
  RM.stop(brake);
  RB.stop(brake);
}

void monitorSetup(){
  Brain.Screen.clearScreen();
  Brain.Screen.printAt(1,20,"LF ");
  Brain.Screen.printAt(1,20,"LB ");
  Brain.Screen.printAt(1,20,"LM ");

  Brain.Screen.printAt(1,20,"RF ");
  Brain.Screen.printAt(1,20,"RM ");
  Brain.Screen.printAt(1,20,"RB ");
} 


void monitorDisplay(){
  if(LF.installed()){
  LF.current(amp);
  LF.temperature(fahrenheit);
  float current=LF.current(amp);
  float temp=LF.temperature(fahrenheit);
  if (temp>=125);
    Brain.Screen.setPenColor(blue);
    Brain.Screen.printAt(20,20, "current = %.2F A Temp = %.0f C",current, temp);
}

else{
  Brain.Screen.printAt(20,20, "motor problem");
}

  if(LF.installed()){
  LF.current(amp);
  LF.temperature(fahrenheit);
  float current=LF.current(amp);
  float temp=LF.temperature(fahrenheit);
  if (temp>=125);
    Brain.Screen.setPenColor(blue);
    Brain.Screen.printAt(20,20, "current = %.2F A Temp = %.0f C",current, temp);
}

else{
  Brain.Screen.printAt(20,20, "motor problem");
}



}
void inchDrive(float target){
  float x = 0.0;
  float error=target;
  float speed=0;
  float kp=1.0;
  float accuracy=0.5;
  LF.setPosition(0.0, rev);
  while(fabs(error) >= accuracy){
    speed=kp*error;
    drive(speed,speed,10);
    x= LF.position(rev)*G*Pi*D;
    error=target - x;
  }
driveBrake();
}

void gyroTurn(float target,  float  b=15){
 float heading =0.0;
float accuracy=2.0;
float error= target - heading;
float kp= 2.0;
float speed = kp * error;
Gyro.setRotation(0.0, degrees);
while 
(fabs(error)>=accuracy){
  speed=kp*error+b*error/fabs(error);
  drive(speed,-speed,10);
  heading=Gyro.rotation();
  error=target - heading;
}
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
  inchDrive(48.0);
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

