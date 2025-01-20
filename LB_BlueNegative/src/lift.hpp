#include "robot.h"
#include "map"

enum LiftStates
{
    idle,
    loading,
    scoring,
    alliance,
    tipping
};

std::map<LiftStates, double> LiftValuesMap = {
    {idle, 0},
    {loading, 76},
    {scoring, 386},
    {alliance, 600},
    {tipping, 750}

};

LiftStates currentState = loading;

void nextState()
{
    switch (currentState)
    {
    case idle:
        currentState = loading;
        break;
    case loading:
        currentState = scoring;
        break;
    case scoring:
        currentState = alliance;
        break;
    case alliance:
        currentState = tipping;
        break;
    case tipping:
        currentState = idle;
    }
}

void prevState()
{
    switch (currentState)
    {
    case idle:
        currentState = tipping;
        break;
    case loading:
        currentState = idle;
        break;
    case scoring:
        currentState = loading;
        break;
    case alliance:
        currentState = scoring;
        break;
    case tipping:
        currentState = alliance;
    }
}

double kp = 0.3;

void liftControl()
{
    double target = LiftValuesMap[currentState];
    double error = target - LBRotation.position(degrees);
    LB.spin(forward, error * kp, volt);
}