#include "robot.h"
#include "map"

enum LiftStates {
    idle,
    loading,
    scoring,
    tipping
};

std::map<LiftStates, double> LiftValuesMap = {
    {idle, 0},
    {loading, 25},
    {scoring, 180},
    {tipping, 230}
};

LiftStates currentState = idle;

void nextState() {
    switch (currentState) {
        case idle:
            currentState = loading;
            break;
        case loading:
            currentState = scoring;
            break;
        case scoring:
            currentState = tipping;
            break;
        case tipping:
            currentState = idle;
    }
}

void prevState() {
    switch (currentState) {
        case idle:
            currentState = tipping;
            break;
        case loading:
            currentState = idle;
            break;
        case scoring:
            currentState = loading;
            break;
        case tipping:
            currentState = scoring;
    }
}

double kp = 5.0;

void liftControl() {
    double target = LiftValuesMap[currentState];
    double error = target - LBRotation.position(degrees);
    LB.spin(forward, error * kp, volt);
}
