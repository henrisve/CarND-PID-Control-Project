#include "PID.h"
#include <iostream>
#include<iomanip>
#include <cmath>

using namespace std;

PID::PID(double Kp_init, double Ki_init, double Kd_init,
        double Kp_init_s, double Kd_init_s, double Kt_init_s) {
    kpid = {Kp_init,Ki_init,Kd_init,
            Kp_init_s, Kd_init_s, Kt_init_s};
    old_CTE = 0;
    total_CTE = 0;
    twiddleLoop = 0;
    twiddleGeneration = 0;
    twiddleError = 0;
    twiddleBestError = INFINITY;
    twiddleBestSpeedError = INFINITY;
    twiddleDP = {Kp_init/10,Ki_init/10,Kd_init/10,
                Kp_init_s/10,Kd_init_s/10,Kt_init_s/10};
    parameterNo=0;
    twiddleCheckNeg=true;
    twiddleWait=true;
    twiddleRuntime=1200;
    twiddleGenrationSinceBetter=0;
}

PID::~PID() {}

double PID::UpdateSteering(double cte){
    total_CTE += cte;

    double steer = -kpid[0] * cte - kpid[2] * ( cte - old_CTE) - kpid[1] * total_CTE;
    //old_CTE = cte;
    if(steer > 1){
        steer = 1; 
    }else if(steer < -1){
        steer = -1; 
    }
    return steer;
}

double PID::UpdateThrottle(double cte, double speed){
    //Note, this is a PD controller, the "third" (kpid[5]) parameter is max_throttle
    double throttle = kpid[5] - kpid[3] * fabs(cte) - kpid[4] * fabs( cte - old_CTE);
    if(throttle > 1){
        throttle = 1; 
    }else if(throttle < -1){
        throttle = -1; 
    }
    old_CTE = cte;
    return throttle;
}

bool PID::twiddle(double cte,double speed){ 
    if(twiddleWait && twiddleLoop<400){ // we get lots of error during the first seconds
        twiddleLoop++;
        return false;
    }else if(twiddleWait){
        cout << "started twiddling" << endl;
        twiddleFromStart=true;
        reset_twiddle(false);
    }
    //cout << "avrageSpeed " << avrageSpeed << " twiddleError "<<twiddleError<< endl;//" speedPenalty "<<speedPenalty<<
    cteError += (fabs(cte)*fabs(cte))/twiddleRuntime;
    avrageSpeed += (speed/twiddleRuntime);
    double speedPenalty = (100-avrageSpeed)/50;
    double *currentBestError;
    bool reset = false;    

    /*
    * Because I both optimize steering and throttle in the same it works
    * better to use two different bestError.
    */
    if(parameterNo < 3){
        twiddleError=cteError;
        currentBestError= &twiddleBestError;
    }else{        
        twiddleError=cteError+speedPenalty;
        currentBestError= &twiddleBestSpeedError;
    }
    
    if(cteError > *currentBestError && twiddleLoop > 10){
        cout << "Error is already too high" ;
        twiddleLoop = twiddleRuntime;
        if(twiddleFromStart){
            cout << ", but because it was from start we give it a second chance" << endl;
            twiddleFromStart=false;
            reset_twiddle(false);
            // add twiddlewait?
            return false;
        }
        if((cte > 3.5 || speed <5) && twiddleLoop > 10){ 
            cout << ", and car too far from track";
            twiddleError= (twiddleRuntime/twiddleLoop)*1.1*twiddleError;
            twiddleLoop = twiddleRuntime;
            reset = true;
        }  
    }
    if((cte > 4 || speed <5) && twiddleLoop > 20){    
        cout << "car is stuck, reset" << endl;
        reset_twiddle(true);
        return true;
    }
    if(twiddleLoop >= twiddleRuntime){
        //will run less and less over time
        if(twiddleGeneration==0){//Only do first time!
            cout << "first time, and got an error of " << twiddleError << ". Next up is to increase P-value for steering!" << endl;
            *currentBestError=twiddleError;
            kpid[0] += twiddleDP[0];
            reset_twiddle(reset);
            twiddleGeneration++;
            return reset;
        }
        cout << endl << "Error this run:" << twiddleError << " , best is " << *currentBestError << endl;
        if(twiddleGenrationSinceBetter > 10){
            /*If we get a worse score in turning due to
              the speed tuning think it does better, we
              migth never reach that score, this will make
              sure we can still tune it.
            */
            cout << "Long time since we got better, let's try increase best_error!" << endl;
            *currentBestError*=1.01;
        }
        if(twiddleError< *currentBestError){
            cout << endl << "new best:" << twiddleError << " when setting param#" << parameterNo << " to " << kpid[parameterNo] <<endl;
            std::cout << std::fixed;
            std::cout << std::setprecision(10);
            cout << "-------------- P-steer    \t I-steer    \t D-steer    \t P-Throttle  \t D-Throttle \t Max-Throttle" << endl;
            cout << "current PID: ";
            for(auto &k:kpid){
            cout << k << " \t";
            }
            cout << endl << "current DP : ";
            for(auto &k:twiddleDP){
            cout << k << " \t";
            }
            cout << endl << endl;
            twiddleGenrationSinceBetter=0;
            *currentBestError=twiddleError;
            twiddleDP[parameterNo] *= 1.1;
            parameterNo = (parameterNo+1) % kpid.size();
            kpid[parameterNo] += twiddleDP[parameterNo];
            twiddleCheckNeg = true;
            //cout << "next parameter #: " << parameterNo  << endl;
        }else{
            if(twiddleCheckNeg){
                cout << "not better, try negative may" << endl;
                kpid[parameterNo] -= 2*twiddleDP[parameterNo];
                twiddleCheckNeg = false;
            }else{
                cout << "still not better, reset and try lower DP for parameter "  << parameterNo << endl;
                kpid[parameterNo] += twiddleDP[parameterNo];  
                twiddleDP[parameterNo] *= 0.9;    
                parameterNo = (parameterNo+1) % kpid.size();
                kpid[parameterNo] += twiddleDP[parameterNo];
                twiddleCheckNeg = true;
                //cout << ", next parameter #:" << parameterNo  << endl;
            }
            
        }
        if(cteError > *currentBestError*1.1)
            reset = true;
        // Do a reset
        twiddleGeneration++;
        twiddleGenrationSinceBetter++;
        twiddleRuntime+=(twiddleGeneration);
        cout << "Generation #" << twiddleGeneration << ". it will run for: " << twiddleRuntime << " steps, on Parameter " << parameterNo << endl;
        reset_twiddle(reset);
        return reset;
    }else{
        twiddleLoop++;
        return false; 
    }
}
void PID::reset_twiddle(bool reset_car){         
    twiddleLoop=0;
    avrageSpeed=0;       
    twiddleError=0;
    cteError=0;
    if(reset_car){
        old_CTE = 0;
        total_CTE = 0;
        twiddleWait=true;
    }else{
        twiddleWait=false;
    }
}

