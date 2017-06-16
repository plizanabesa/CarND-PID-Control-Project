#include "PID.h"
#include <uWS/uWS.h>
#include <math.h>

using namespace std;
using std::vector;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
    
  // Initialize error functions
  p_error = 0;
  i_error = 0;
  d_error = 0;
    
  // Initialize twiddle parameters
  dp = Kp/4;
  di = Ki/4;
  dd = 0.1;
  best_error = 1e9;
  norm_error = 0;
  total_error = 0;
  num_steps = 0;
}

void PID::UpdateError(double cte) {
  d_error = cte-p_error;
  p_error = cte;
  i_error += cte;
    
  num_steps++;
  total_error += fabs(-Kp*p_error - Ki*i_error - Kd*d_error);
  //total_error += pow(cte,2);

  std::cout << "p_error: " << p_error << " d_error: " << d_error
    << " i_error: " << i_error << " steps: " << num_steps << std::endl;
}

double PID::TotalError() {
  return -Kp*p_error - Ki*i_error - Kd*d_error;
}

void PID::Restart(uWS::WebSocket<uWS::SERVER> ws){
    std::string reset_msg = "42[\"reset\",{}]";
    ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}

void PID::Twiddle(double tolerance){
    double norm_error = total_error/num_steps;
    total_error = 0;
    num_steps = 0;
    
    std::cout << "Starting twiddle with Kp: " << Kp << " Kd: " << Kd
    << " Ki: " << Ki << " Error: " << norm_error << " Best Error: "
    << best_error << " dp: " << dp << " di: " << di
    << " dd: " << dd << " Sum d: " << dp + di + dd << std::endl;
    
    /*if (best_error > tolerance){
        best_error = norm_error;
        
        Kp += dp;
        Ki += di;
        Kd += dd;
        return
    }*/
    
    // Reset i d and p errors
    p_error = 0;
    i_error = 0;
    d_error = 0;
    
    Kp += dp;
    Ki += di;
    Kd += dd;
    
    if (norm_error < best_error){
        // Normalized error has decreased: best error increase delta p,i,d
        best_error = norm_error;
        
        dp *= 1.1;
        di *= 1.1,
        dd *= 1.1;
        
        std::cout << "Increase Kp: " << Kp << " Kd: " << Kd
          << " Ki: " << Ki << " Error: " << best_error
          << " Sum d: " << dp + di + dd << std::endl;
    }
    else{
        // Normalized error higher than best error
        
        // Decrease hyperm-parameters by two fold
        Kp -= 2 * dp;
        Ki -= 2 * di;
        Kd -= 2 * dd;
        
        // Decrease delta parameters
        dp *= 0.9;
        di *= 0.9,
        dd *= 0.9;
        
        std::cout << "Decrease Kp: " << Kp << " Kd: " << Kd
          << " Ki: " << Ki << " Error: " << norm_error << " Best Error: "
          << best_error << " Sum d: " << dp + di + dd << std::endl;
        
    }
}

