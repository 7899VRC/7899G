#include "robot.h"

bool verified = false;
bool step_one_complete = false;
bool step_two_complete = false;

void verification() {
    if (Controller.ButtonY.pressing()) {
        step_one_complete = true;
    }
    if (Controller.ButtonLeft.pressing()) {
        if (step_one_complete == true) {
            step_two_complete = true;
        }
    }
    if (Controller.ButtonRight.pressing()) {
        if (step_two_complete == true) {
            verified = true;
        }
    }
    if (Controller.ButtonDown.pressing()) {
        step_one_complete = false;
        step_two_complete = false;
        verified = false;
    }
    
}