#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <vector>

// for convenience
using json = nlohmann::json;
using namespace std;

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

// twiddle python implementation

/*def twiddle(tol=0.2): 
    p = [0, 0, 0]
    dp = [1, 1, 1]
    robot = make_robot()
    x_trajectory, y_trajectory, best_err = run(robot, p)

    it = 0
    while sum(dp) > tol:
        print("Iteration {}, best error = {}".format(it, best_err))
        for i in range(len(p)):
            p[i] += dp[i]
            robot = make_robot()
            x_trajectory, y_trajectory, err = run(robot, p)

            if err < best_err:
                best_err = err
                dp[i] *= 1.1
            else:
                p[i] -= 2 * dp[i]
                robot = make_robot()
                x_trajectory, y_trajectory, err = run(robot, p)

                if err < best_err:
                    best_err = err
                    dp[i] *= 1.1
                else:
                    p[i] += dp[i]
                    dp[i] *= 0.9
        it += 1
    return p*/

bool twiddle_on = false;
double best_error = 1000000;
int twiddle_index = 0;
int twiddle_state = 0;
int twiddle_iterations = 0;

std::vector<double> p = {0.5, 0.5, 0.5};
std::vector<double> dp = {0.1, 0.1, 0.1};

void twiddle(PID &pid_control) {
  cout << "PID ERRPR: " << pid_control.TotalError() << endl;
  cout << "BEST ERROR: " << best_error << endl;
  
  if (twiddle_state == 0) {
    best_error = pid_control.TotalError();
    p[twiddle_index] += dp[twiddle_index];
    twiddle_state = 1;
  } else if (twiddle_state == 1) {
    if (pid_control.TotalError() < best_error) {
      best_error = pid_control.TotalError();
      dp[twiddle_index] *= 1.1;
      twiddle_index = (twiddle_index + 1) % 3;
      p[twiddle_index] += dp[twiddle_index];
      twiddle_state = 1;
    } else {
      p[twiddle_index] -= 2 * dp[twiddle_index];
      // coefficient cannot be smaller than 0
      if (p[twiddle_index] < 0) {
        p[twiddle_index] = 0;
        twiddle_index = (twiddle_index+ 1) % 3;
      }
      twiddle_state = 2;
    }
  } else {
    if (pid_control.TotalError() < best_error) {
      best_error = pid_control.TotalError();
      dp[twiddle_index] *= 1.1;
      twiddle_index = (twiddle_index + 1) % 3;
      p[twiddle_index] += dp[twiddle_index];
      twiddle_state = 1;
    } else {
      p[twiddle_index] += dp[twiddle_index];
      dp[twiddle_index] *= 0.9;
      twiddle_index = (twiddle_index + 1) % 3;
      p[twiddle_index] += dp[twiddle_index];
      twiddle_state = 1;
    }
  }

  pid_control.Init(p[0], p[1], p[2]);

}


int main()
{
  uWS::Hub h;

  PID pid;

  // TODO: Initialize the pid variable.
  // Here I manually tune the PID parameters
  // P will control how fast the car converrge to the center but overshot if it's too large
  // I will caligrate the drift over the time
  // D will decrease the overshot by P and make it converge more smooth
  // By trial and error and I find following numbers that works in the simulator

  pid.Init(0.15, 0.001, 1.0);

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          double steer_value;
          double required_speed = 30.0;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          pid.UpdateError(cte);
          steer_value = pid.TotalError();

          if (steer_value > 1.0) { steer_value = 1.0; }
          else if (steer_value < - 1.0) { steer_value = -1.0; }

          


          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
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
