/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:      george kirkman copy of egor code                                               */
/*    Created:      9/26/2024, 1:19:54 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
brain Brain;
 controller Controller1;
 motor LF=motor(PORT13,ratio6_1,true);
 motor LB=motor(PORT14,ratio6_1,true);
  motor LM=motor(PORT2,ratio6_1,true);
 motor RF=motor(PORT3,ratio6_1,false); 
 motor RB=motor(PORT4,ratio6_1,false);
motor RM=motor(PORT5,ratio6_1,false);
inertial Gyro = inertial (PORT10);
float D = 3.25;
float Pi= 3.14;
float G = 36.0/60.0;

void monitorSetup(){
  Brain.Screen.clearScreen();
  Brain.Screen.printAt(1,20,"LF ");
  Brain.Screen.printAt(1,40,"LM ");
  Brain.Screen.printAt(1,60,"LB ");

  Brain.Screen.printAt(1,80,"RF ");
  Brain.Screen.printAt(1,100,"RM ");
  Brain.Screen.printAt(1,120,"RB ");
}

void monitorDisplay(){
  int y =20;
  if(LF.installed()){
  float current=LF.current(amp);
  float temp=LF.temperature(celsius);
  if (temp>50){
    Brain.Screen.setPenColor(red);
  }
  Brain.Screen.printAt(20,20, "current = %.2f A Temp = %.0f  C",current, temp);
}
else{
  Brain.Screen.printAt(20,20, "MOTOR PROBLEM !!!              ");
}
 Brain.Screen.setPenColor(white);

y=y+20;
if(LM.installed()){
  current=LM.current(amp);
  temp=LM.temperature(celsius);
  if (temp>50){
    Brain.Screen.setPenColor(red);
  }
  Brain.Screen.printAt(20,y, "current = %.2f A Temp = %.0f  C",current, temp);
}
else{
  Brain.Screen.printAt(20,y, "MOTOR PROBLEM !!!              ");
}
 Brain.Screen.setPenColor(white);
}

void drive(int lspeed, int rspeed, int wt){
LF.spin(forward, lspeed, percent);
LB.spin(forward, lspeed, percent);
LM.spin(forward, rspeed, percent);
RF.spin(forward, rspeed, percent);
RB.spin(forward, rspeed, percent);
RM.spin(forward, rspeed, percent);
wait(wt,msec);
 }


void driveVolts(int lspeed, int rspeed, int wt){
 lspeed=lspeed*120;
 rspeed=rspeed*120;
 LF.spin(forward, lspeed, voltageUnits::mV);
LB.spin(forward, lspeed, voltageUnits::mV);
LM.spin(forward, lspeed, voltageUnits::mV);
RF.spin(forward, rspeed, voltageUnits::mV);
RB.spin(forward, rspeed, voltageUnits::mV);
RM.spin(forward, rspeed, voltageUnits::mV);
wait(wt,msec);
}
void drivebreak(){
LF.stop(brake);
LB.stop(brake);
LM.stop(brake);
RF.stop(brake);
RB.stop(brake);
RM.stop(brake);
}
 void brakedrive();
void inchDrive(float target){
float x =0.0;
float error=target;
float speed=0;
float kp=1.0;
float accuracy=1.0;
while (fabs(error) >= accuracy){
speed=kp*error;
drive(speed,speed,10);
x=LF.position(rev)*G*Pi*D;
error= target - x;
}

drivebreak();
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
drivebreak();
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
   int lstick = Controller1.Axis3.position(percent);
   int rstick = Controller1.Axis2.position(percent);
   int stick4 = Controller1.Axis4.position(percent);
driveVolts(lstick+stick4,lstick-stick4,10);
  }
}
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
