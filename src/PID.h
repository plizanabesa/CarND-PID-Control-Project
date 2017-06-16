#include <vector>
#include <uWS/uWS.h>

#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  double total_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;
    
  /*
  * Twiddle Coefficients
  */
  double dp;
  double di;
  double dd;
  double best_error;
  int num_steps;
  double norm_error;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  * Restart the simulator
  */
  void Restart(uWS::WebSocket<uWS::SERVER> ws);

  int getNumSteps () { return num_steps;}
    
  /*
  * Twiddle to optimize PID hyper-parameters
  */
  void Twiddle(double tolerance);
};

#endif /* PID_H */
