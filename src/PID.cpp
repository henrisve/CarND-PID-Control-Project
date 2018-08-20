#include "PID.h"
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID(double Kp_init, double Ki_init, double Kd_init) {
    Kp=Kp_init;
    Ki=Ki_init;
    Kd=Kd_init;
    old_CTE = 0;
    total_CTE = 0
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
}

double PID::UpdateSteering(double cte){
    total_CTE += cte;
    double steer = -Kp * cte - Kd * (cte - old_CTE) - Ki * total_CTE;
    old_CTE = cte;
    if(steer > 1){
        steer = 1;
        cout << "steer larger than 1" << endl;
    }else if(steer < -1){
        steer = -1;
        cout << "steer less than -1" << endl;
    }
    return steer;
}

double PID::UpdateThrottle(double speed, double maxspeed, double steer){
    //Todo, pid for speed
    double speed_diff = maxspeed*(1-(steer*0.5)) - speed;
    return 1;
}

void PID::UpdateError(double cte) {
}

double PID::TotalError() {
}

