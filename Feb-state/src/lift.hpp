
#include "robot.h"
#include "map"

enum LiftStates
{
    idle,
    loading,
    scoring,
    loading2,
    alliance,
    tipping
};

std::map<LiftStates, double> LiftValuesMap = {
    {idle, 0},
    {loading, 53},
    {loading2, 200},
    {scoring, 386},
    {alliance, 630},
    {tipping, 750}

};

LiftStates currentState = idle;

void nextState()
{
    switch (currentState)
    {
    case idle:
        currentState = loading;
        break;
    case loading:
        currentState = loading2;
        break;
    case loading2:
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
    case loading2:
        currentState = loading;
        break;
    case scoring:
        currentState = loading2;
        break;
    case alliance:
        currentState = scoring;
        break;
    case tipping:
        currentState = alliance;
    }
}

double kp = 0.2;

void liftControl()
{
    double target = LiftValuesMap[currentState];
    double error = target - LBRotation.position(degrees);
    LB.spin(forward, error * kp, volt);
}
