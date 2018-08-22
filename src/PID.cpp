#include "PID.h"
#include <iostream>
#include <cmath>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID(double Kp_init, double Ki_init, double Kd_init) {
    kpid = {Kp_init,Ki_init,Kd_init};
    //kpid = Ki_init;
    //kpid = Kd_init;
    old_CTE = 0;
    total_CTE = 0;
    twiddleLoop = 0;
    twiddleGeneration = 0;
    twiddleError = 0;
    twiddleBestError = INFINITY;
    twiddleDP = {Kp_init/10,Ki_init/10,Kd_init/10};
    parameterNo=0;
    twiddleCheckNeg=true;
    twiddleFromStart=true;
    twiddleRuntime=1200;
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
}

double PID::UpdateSteering(double cte){
    total_CTE += cte;

    double steer = -kpid[0] * cte - kpid[2] * ( cte - old_CTE) - kpid[1] * total_CTE;
    //cout << "steer = " << -kpid[0] << "*" << cte << -kpid[2] << "* (" << old_CTE << "-"<<  cte<<")"<< -kpid[1]<< "*"<< total_CTE;
    old_CTE = cte;
    if(steer > 1){
        //cout << "steer is larger than 1: " << steer << endl;
        steer = 1; 
    }else if(steer < -1){
        //cout << "steer less than -1 : " << steer << endl;
        steer = -1; 
    }
    return steer;
}

double PID::UpdateThrottle(double speed, double maxspeed, double steer){
    //Todo, pid for speed
    double speed_diff = maxspeed*(1-(steer*0.5)) - speed;
    return 1;
}

bool PID::twiddle(double cte){ 
    if(twiddleFromStart && twiddleLoop<400){ // we get lots of error during the first seconds
        twiddleLoop++;
        return false;
    }else if(twiddleFromStart){
        cout << "started looking" << endl;
        twiddleError=0;
        twiddleLoop=0;
        twiddleFromStart=false;
    }
    twiddleError+=(fabs(cte)*fabs(cte))/twiddleRuntime;
    
    bool reset = false;    
    if(twiddleError > twiddleBestError){
        cout << "error too high " ;
        //twiddleError= (runTime/twiddleLoop)*2*twiddleError;
        twiddleLoop = twiddleRuntime;
        if(cte > 3 && twiddleLoop > 10){ 
    //<----- this if was outside earlier, but the hard turn after
            // the bridge seems to return waay to high values
            // wheels on the side on other parts is about 1.5, where
            // at that point its closer to 3-4
            cout << "and car too far from track: ";
            twiddleError= (twiddleRuntime/twiddleLoop)*2*twiddleError;
            twiddleLoop = twiddleRuntime;
            reset = true;
        }  
    }
    
    if(twiddleLoop >= twiddleRuntime){
        //will run less and less over time
        cout << "error:" << twiddleError << " , best is " << twiddleBestError << endl;
        if(twiddleGeneration==0){//Only do first time!
            cout << "first time" << endl;
            twiddleBestError=twiddleError;
            kpid[0] += twiddleDP[0];
            twiddleLoop=0;
            if(reset){
                old_CTE = 0;
                total_CTE = 0;
                twiddleFromStart=true;
            }
            twiddleGeneration++;
            twiddleError=0;
            return reset;
        }
        if(twiddleError< twiddleBestError){
            cout << "new best:" << twiddleError << endl;
            twiddleBestError=twiddleError;
            twiddleDP[parameterNo] *= 1.1;
            parameterNo = (parameterNo+1) % kpid.size();
            kpid[parameterNo] += twiddleDP[parameterNo];
            twiddleCheckNeg = true;
            cout << "next parameter #: " << parameterNo  << endl;
        }else{
            if(twiddleCheckNeg){
                cout << "not better, try negative may" << endl;
                kpid[parameterNo] -= 2*twiddleDP[parameterNo];
                twiddleCheckNeg = false;
            }else{
                cout << "still not better, reset and try lower dp ";
                kpid[parameterNo] += twiddleDP[parameterNo];  
                twiddleDP[parameterNo] *= 0.9;    
                parameterNo = (parameterNo+1) % kpid.size();
                kpid[parameterNo] += twiddleDP[parameterNo];
                twiddleCheckNeg = true;
                cout << ", next parameter #:" << parameterNo  << endl;
            }
            
        }
        if(twiddleError > twiddleBestError*1.1)
            reset = true;
        // Do a reset
        twiddleLoop=0;
        twiddleGeneration++;
        twiddleRuntime=twiddleRuntime+twiddleGeneration*10;
        twiddleError=0;
        cout << "current PID: " <<kpid[0] << " \t"  << kpid[1] << " \t"  << kpid[2] << " \t" << endl;
        cout << "current DP : " twiddleDP[0] << " \t" << twiddleDP[1] << " \t" << twiddleDP[2] << " \t" << endl;
        if(reset){
            old_CTE = 0;
            total_CTE = 0;
            twiddleFromStart=true;
        }
        return reset;
    }else{
        twiddleLoop++;
        return false; 
    }

}

void PID::UpdateError(double cte) {
}

double PID::TotalError() {
}

