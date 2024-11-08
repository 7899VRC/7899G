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

motor conveyorA = motor(PORT18, ratio6_1, true);
motor conveyorB = motor(PORT15, ratio6_1, true);
motor_group conveyor = motor_group(conveyorA, conveyorB);
motor lift = motor(PORT19, ratio18_1, false);

digital_out mogo_mech = digital_out(Brain.ThreeWirePort.A);

inertial Inertial = inertial(PORT13);

//bool mogo_mech_bool = true;
bool verified = false;

// PID Variables
double previousErrorDistance = 0;
double integralDistance = 0;
double previousErrorHeading = 0;
double integralHeading = 0;
const double wheelDiameter = 2.75; // Wheel diameter in inches
const double WHEELCIRCUMFERENCE = wheelDiameter * 3.141592653589793; // Circumference in inches
const double PI = 3.1415;
const double D = 2.75;
const double G = 3.0 / 4.0;

void mogo_mech_control() {
  
  mogo_mech.set(!mogo_mech.value());
}

void driveVolts(int lspeed, int rspeed, int wt){
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

void driveBrake(){
  left_motor1.stop(brake);
  left_motor2.stop(brake);
  left_motor3.stop(brake);
  right_motor1.stop(brake);
  right_motor2.stop(brake);
  right_motor3.stop(brake);
}

//InchDrive helped by Jeffrey
void inchDrive(float targetDistanceInches, double targetVelocity, float timeout, float wait_ms=10, float kp = 5.0){
  //float targetDistanceInDegrees = (targetDistanceInches/WHEELCIRCUMFERENCE) * 360; //converts distance to degrees
  left_motor1.setPosition(0.0 , rev); //resets rotations of left_motor1
  float actualDistance = 0.0; //actual distance calculates rotations of left_motor1
  float error=targetDistanceInches-actualDistance;
  
  float accuracy=0.5;
  timer t2;

  while(t2.time(msec) < timeout){
    targetVelocity=kp*error;
    //Drives forward until actual distance meets target distance
   
      driveVolts(targetVelocity,targetVelocity,100);
   
    
    actualDistance = left_motor1.position(rev)*PI*D*G;
    error=targetDistanceInches-actualDistance;
  Brain.Screen.print(error);
  Brain.Screen.newLine();
  }
  driveBrake();
  wait(wait_ms, msec);
} 

void gyroTurn(float targetHeading, int timeout, double kp = 2){

	float heading=0.0; //initialize a variable for heading
	// Inertial.setRotation(0.0, degrees);  //reset Gyro to zero degrees
      Inertial.resetRotation();

	float error=targetHeading-heading;
  float accuracy=2.0;
  
  timer t1;


	while(t1.time(msec) < timeout){
		heading=Inertial.rotation(deg);  //measure the heading of the robot
    error=targetHeading-heading;
    float speed = error * kp;
    // if(error>0){
		driveVolts(speed, -speed, 10); //turn right at speed
    // }
    // else{
    //   driveVolts(speed, -speed, 10); //turn left at speed
    // }
    // wait(20,msec);

	}
	driveBrake();  //stop the drive
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
      Inertial.calibrate();
      while (Inertial.isCalibrating()) {
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


void autonomous(void){
      
  inchDrive(-10, 80, 500, 0, 8);
  
  mogo_mech.set(true);
  wait(200, msec);
  conveyor.setVelocity(100, pct);
  conveyor.setMaxTorque(100, pct);
  conveyor.spin(fwd);
  gyroTurn(66, 700);
  inchDrive(22, 80, 1200, 0);
  wait(300, msec);
  inchDrive(14, 80, 600, 0);
  wait(500, msec);
  gyroTurn(120, 1000);
  
  inchDrive(-14, 80, 800, 0);
  
  mogo_mech.set(false);
  wait(80, msec);
  inchDrive(10.5, 80, 800, 0); //come out of corner
  gyroTurn(-119, 1000);
  inchDrive(-85, 80, 2000, 20, 2);
  mogo_mech.set(true);
  wait(50, msec);
  gyroTurn(177, 1200);
  inchDrive(22, 80, 1200, 0);
  wait(200, msec);
  inchDrive(14, 80, 600, 10);
  wait(50, msec);
  gyroTurn(-120, 1000);
  inchDrive(30, 80, 800);
  gyroTurn(-45, 500);


  
  
  
  
  
  
  // conveyor.stop();
      // inchDrive(-5, 100, 800, 20);
      // mogo_mech.set(true);
      // wait(500,msec);
      // conveyor.setVelocity(100, pct);
      // conveyor.setMaxTorque(100, pct);
      // conveyor.spin(fwd);
      // wait(2, sec);
      // conveyor.stop();
      // gyroTurn(105, 500);
      // inchDrive(-50, 50, 1000, 10);
      // mogo_mech.set(false);
      // inchDrive(10, 80, 800, 10);
      // inchDrive(-12, 100, 800, 10, 5); 
      // inchDrive(70, 10, 800, 10, 2);
       
      // gyroTurn(-30, 300);
      // inchDrive(100, 10, 800, 0.5);
      
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

  conveyorA.setBrake(coast);
  conveyorB.setBrake(coast);
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


  wait(10,msec); 

  
  


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
