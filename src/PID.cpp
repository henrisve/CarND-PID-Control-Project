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
    total_CTE = 0;
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
}

double PID::UpdateSteering(double cte){
    total_CTE += cte;
    double steer = -Kp * cte - Kd * ( old_CTE - cte) - Ki * total_CTE;
    cout << "steer = " << -Kp << "*" << cte << -Kd << "* (" << old_CTE << "-"<<  cte<<")"<< -Ki<< "*"<< total_CTE;
    old_CTE = cte;
    if(steer > 1){
        cout << "steer is larger than 1: " << steer << endl;
        steer = 1; 
    }else if(steer < -1){
        cout << "steer less than -1 : " << steer << endl;
        steer = -1; 
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

