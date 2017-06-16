#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

/* References
https://discussions.udacity.com/t/how-to-make-the-pid-output-normalized-to-be-within-1-1/252173/3
https://discussions.udacity.com/t/twiddle-application-in-pid-controller/243427/9
https://github.com/jendrikjoe/UdacityProjects/blob/master/src/main.cpp
Q&A David Silver*/

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;
  /*double current_time;
  double previous_time;
  double dt = 0;*/
  PID steer_pid;
  PID speed_pid;
  steer_pid.Init(0.12, 0.00025, 1.5);
  speed_pid.Init(.12 , 0.0001 , 0.8);

  h.onMessage([&steer_pid,&speed_pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double current_time = clock();
          double clocks_per_sec = CLOCKS_PER_SEC;
          bool use_steer_twiddle = true;
          bool use_speed_twiddle = true;
            
          // If CTE exceeds value, restart simulator and Twiddle
          if (std::abs(cte) > 2.2 && steer_pid.getNumSteps() > 100 && use_steer_twiddle){
              std::cout << "Starting steer twiddle: " << std::endl;
              steer_pid.Twiddle(1e8);
          }
            
          if (std::abs(cte) > 2.2 && speed_pid.getNumSteps() > 250 && use_speed_twiddle){
              std::cout << "Starting speed twiddle: " << std::endl;
              speed_pid.Twiddle(1e8);
          }

          if (std::abs(cte) > 5){
            steer_pid.Restart(ws);
          }
            
          if (steer_pid.getNumSteps() > 750 && use_steer_twiddle){
              std::cout << "Starting steer twiddle: " << std::endl;
              steer_pid.Twiddle(1e8);
          }
          
          if (speed_pid.getNumSteps() > 750 && use_speed_twiddle){
              std::cout << "Starting speed twiddle: " << std::endl;
              speed_pid.Twiddle(1e8);
          }
            
          // Update steer angle using steer PID
          steer_pid.UpdateError(cte);
          double steer_value = steer_pid.TotalError();
          if (steer_value < -1)
            steer_value = -1;
          else if (steer_value > 1)
            steer_value = 1;
        
          // Update speed throttle using speed PID
          double target_speed = 25.*(1.-fabs(steer_value)) + 20.;
          speed_pid.UpdateError(speed - target_speed);
          double throttle_value = speed_pid.TotalError();
          /*if (throttle_value < -1)
            throttle_value = -1;
          else if (throttle_value > 1)
            throttle_value = 1;*/
            
          // DEBUG
          std::cout << "CTE: " << cte << " Steering: " << steer_value
            << " Speed: " << speed << " Target Speed: " << target_speed << " Throttle: " << throttle_value << std::endl;
        
          /*dt: " << current_time
            << " Clocks: " << clocks_per_sec <<std::endl;*/

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          //msgJson["throttle"] = 0.2;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
