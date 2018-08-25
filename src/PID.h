#ifndef PID_H
#define PID_H
#include <vector>
class PID {
  
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  //double Kp;
  //double Ki;
  //double Kd;
  std::vector<double> kpid;
  double old_CTE;
  double total_CTE;

  /*
  * twiddle
  */

  int twiddleLoop;
  std::vector<double> twiddleDP;
  double twiddleError;
  int twiddleGeneration;
  double twiddleBestError;
  double twiddleBestSpeedError;
  int parameterNo;
  bool twiddleCheckNeg;
  bool twiddleFromStart;
  bool twiddleWait;
  int twiddleRuntime;
  double avrageSpeed;
  double cteError;
  int twiddleGenrationSinceBetter;
  /*
  * Constructor
  */
PID(double Kp_init, double Ki_init, double Kd_init,
    double Kp_init_s, double Kd_init_s, double Kt_init_s);

  /*
  * Destructor.
  */
  virtual ~PID();

  double UpdateSteering(double cte);
  double UpdateThrottle(double cte, double speed);
  bool twiddle(double cte, double speed, double throttle);
  void reset_twiddle(bool reset_car);

};

#endif /* PID_H */
