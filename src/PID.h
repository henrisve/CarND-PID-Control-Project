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
  

  /*
  * Other
  */
  double old_CTE;
  double total_CTE;
  int twiddleLoop;
  std::vector<double> twiddleDP;
  double twiddleError;
  int twiddleGeneration;
  double twiddleBestError;
  int parameterNo;
  bool twiddleCheckNeg;
  bool twiddleFromStart;
  int twiddleRuntime;

  /*
  * Constructor
  */
  PID(double Kp_init, double Ki_init, double Kd_init);

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);


  double UpdateSteering(double cte);
  double UpdateThrottle(double speed, double maxspeed, double steer);
  bool twiddle(double cte);
  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */
