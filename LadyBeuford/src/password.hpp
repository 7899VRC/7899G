#include "robot.h"

bool verified = false;
bool p1 = false;

void password()
{
    if (p1 == true and Controller.ButtonY.pressing() == true)
    {
        verified = true;
    }
    else if (Controller.ButtonRight.pressing() == true)
    {
        p1 = true;
    }
}