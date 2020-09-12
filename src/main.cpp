#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"
#include "helpers.h"


// For convenience
using nlohmann::json;
using std::string;


int main() {
  
  uWS::Hub h;

 
  // Initialize the pid instances
  PID steer_pid;
  PID speed_pid;
  
  steer_pid.init(0.15, 0.0001, 3.5);
  speed_pid.init(0.3, 0.0004, 1);
  
  
  h.onMessage([&steer_pid, &speed_pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          // double angle = std::stod(j[1]["steering_angle"].get<string>());
          
          double steer_value, throttle;
          double target_speed = 30.0;
          
          // Update error params and generate new values
          steer_pid.updateError(cte);
          speed_pid.updateError(speed - target_speed);
          
          steer_value = steer_pid.totalError();
          throttle = speed_pid.totalError();
          
          // If tolerance not met then twiddle params
          if(!steer_pid.toleranceMet())
          {    
              std::vector<double> new_params = steer_pid.twiddle(cte, 0.1);
              steer_pid.updateKp(new_params[0]);
              steer_pid.updateKi(new_params[1]);
              steer_pid.updateKd(new_params[2]);
              steer_pid.printParams();
          }
          
          if(!speed_pid.toleranceMet())
          {    
              std::vector<double> new_params = speed_pid.twiddle(cte, 0.1);
              speed_pid.updateKp(new_params[0]);
              speed_pid.updateKi(new_params[1]);
              speed_pid.updateKd(new_params[2]);
              speed_pid.printParams();
          }
          
          
          if(steer_pid.toleranceMet())  
          {
              std::cout << "STEER TOLERANCE MET" << std::endl;
              steer_pid.printParams(); 
          } 
          
          if(speed_pid.toleranceMet())  
          {
              std::cout << "SPEED TOLERANCE MET" << std::endl;
              speed_pid.printParams(); 
          } 
          
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, 
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}