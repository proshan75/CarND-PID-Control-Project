#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

//////////////////////////////////////////////////
// Set twiddle flag for PID parameter optimization
bool RUN_TWIDDLE = true;
//////////////////////////////////////////////////

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s)
{
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos)
  {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos)
  {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

double normalizeError(double error)
{
  // cout << __func__ << " input error: " << error;
  double n_error = std::fmod(error, (2 * pi()));

  if (n_error < -pi())
    n_error += 2 * pi();
  else if (n_error > pi())
    n_error -= 2 * pi();

  n_error = ((n_error + pi()) / pi()) - 1.0;
  // cout << " normalized error: " << n_error << endl;
  return n_error;
}

int main(int argc, char *argv[])
{
  uWS::Hub h;

  PID pid;
  // Initialize the pid variable.

  double kp = std::stod(argv[1]), ki = std::stod(argv[2]), kd = std::stod(argv[3]), tol = std::stod(argv[4]);
  if (RUN_TWIDDLE)
  {
    pid.Init(kp, ki, kd, tol);
  }
  else
  {
    pid.Init(kp, ki, kd);
  }

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "")
      {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry")
        {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value = 0.0;
          /*
          * TODO: Calculate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          // std::cout << "CTE: " << cte << " Speed: " << speed << " Angle: " << angle << std::endl;

          if (RUN_TWIDDLE)
          {
            pid.UpdateError(cte);

            if (pid.Current_iteration < pid.MAX_TWIDDLE_ITERATIONS) // && RUN_TWIDDLE
            {
              pid.AddError(cte);
              pid.Current_iteration++;
            }
            else
            {
              double error = pid.CalculateError(pid.Current_iteration);
              if (pid.ShouldRunTwiddle())
              {
                pid.Current_index %= 3;
                pid.Twiddle(error);
                pid.Current_index++;
              }
              else
              {
                // best parameter values achieved as follows
                cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << endl;
                cout << "Improved p: " << pid.Kp << " i: " << pid.Ki << " d: " << pid.Kd << endl;
                cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << endl;
                RUN_TWIDDLE = false;
              }

              pid.ResetTwiddle();
              std::string msg2 = "42[\"reset\",{}]";
              ws.send(msg2.data(), msg2.length(), uWS::OpCode::TEXT);
            }

            steer_value = normalizeError(pid.TotalError());
            // std::cout << "CTE: " << cte << " Steering Value: " << steer_value << " Speed: " << speed << " Angle: " << angle << std::endl;

            json msgJson;
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = 0.3;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
          else
          {
            pid.UpdateError(cte);
            steer_value = normalizeError(pid.TotalError());
            // DEBUG
            std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

            json msgJson;
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = 0.3;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
        }
      }
      else
      {
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
