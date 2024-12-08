#ifndef ROBOT
#define ROBOT
#include "vex.h"

using namespace vex;

// competition Competition;
// brain Brain;
// controller Controller;

// // Left motors
// motor left_motor_front;
// motor left_motor_middle;
// motor left_motor_back;

// // Right motors
// motor right_motor_front;
// motor right_motor_middle;
// motor right_motor_back;

// digital_out mogo_mech;

// motor hook;
// motor LB;

// inertial Inertial;
// optical Optical;
// color detectedColor;
// rotation LBRotation;

competition Competition;
brain Brain;
controller Controller;

// Left motors
motor left_motor_front = motor(PORT8, ratio6_1, false);
motor left_motor_middle = motor(PORT9, ratio6_1, true);
motor left_motor_back = motor(PORT21, ratio6_1, true);

// Right motors
motor right_motor_front = motor(PORT20, ratio6_1, true);
motor right_motor_middle = motor(PORT18, ratio6_1, false);
motor right_motor_back = motor(PORT17, ratio6_1, false);

digital_out mogo_mech = digital_out(Brain.ThreeWirePort.A);

motor hook = motor(PORT2, ratio6_1, false);
motor LB = motor(PORT4, ratio36_1, true);

inertial Inertial = inertial(PORT13);
optical Optical = optical(PORT15);
color detectedColor = Optical.color();
rotation LBRotation = rotation(PORT3, false);

#endif