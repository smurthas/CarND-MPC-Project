#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;
long long now() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

/*vector<double> world_to_vehicle_x(double v_x, double v_y, double v_psi, double x, double y) {
  double dx_map = x - v_x;
  double dy_map = y - v_y;
  double f = -1.0 * psi1;
  double x_vehicle = dx_map*cos(f) - dy_map*sin(f);
  double y_vehicle = dy_map*cos(f) + dx_map*sin(f);
}*/

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

//vector<double> world_to_vehicle()

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
/*double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}*/

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main(int argc, char* argv[]) {
  uWS::Hub h;
  int N = 6;
  if (argc > 1) {
    N = std::atoi(argv[1]);
  }
  double dt = 0.2;
  if (argc > 2) {
    dt = std::atof(argv[2]);
  }
  int actuator_delay = 100;
  if (argc > 3) {
    actuator_delay = std::atoi(argv[3]);
  }

  // MPC is initialized here!
  MPC mpc(N, dt);

  // keep track of previous a and delta value to just in the next time step for
  // handling actuator delay
  double prev_a = 0.0;
  double prev_delta = 0.0;

  h.onMessage([&](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = double(j[1]["speed"]) * 0.44704;

          // estimate the solver time delay, ~30 ms for N=6, increase w N^2
          double solve_delay = ((30 * pow(N, 2.0))/pow(6, 2.0));
          // estimate total delay as solver + actuators
          double delay_s = (solve_delay + actuator_delay) / 1000.0;

          double x1 = (px + v * cos(psi) * delay_s);
          double y1 = (py + v * sin(psi) * delay_s);
          double psi1 = (psi + v * prev_delta / mpc.Lf * delay_s);
          double v1 = (v + prev_a * delay_s);
          std::cout <<"px: "<<px<<", py: "<<py<<", psi: "<<psi<<", v: "<<v<<std::endl;
          std::cout <<"x1: "<<x1<<", y1: "<<y1<<", psi1: "<<psi1<<", v1: "<<v1
                    <<", delay_s: "<<delay_s<< std::endl;
          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          Eigen::VectorXd state(4);
          state << 0, 0, 0, v1;

          Eigen::VectorXd xvals(ptsx.size());
          Eigen::VectorXd yvals(ptsy.size());
          for (int i = 0; i < ptsx.size(); i++) {
            // TODO: convert to vehicle coords?
            //double theta = psi + 1.57; // rotation from map coords
            double dx_map = ptsx[i] - x1;
            double dy_map = ptsy[i] - y1;
            double f = -1.0 * psi1;
            double x_vehicle = dx_map*cos(f) - dy_map*sin(f);
            double y_vehicle = dy_map*cos(f) + dx_map*sin(f);
            xvals[i] = x_vehicle;
            yvals[i] = y_vehicle;
            next_x_vals.push_back(x_vehicle);
            next_y_vals.push_back(y_vehicle);
          }
          auto coeffs = polyfit(xvals, yvals, 3);

          //long long sstart = now();
          vector<double> output = mpc.Solve(state, coeffs);
          //long long send = now();
          //std::cout<< "solve ET: " << (send-sstart) << std::endl;

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          prev_delta = min(max(output[0], -0.4363), 0.4363);
          prev_a = output[1];
          double steer_value = -1.0 * (prev_delta/0.436332);
          //double steer_value = -1.0/25.0;
          double throttle_value = output[1];
          //double throttle_value = 0.3;

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          //msgJson["steering_angle"] = 0.1;
          //msgJson["throttle"] = 0;;

          //Display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          int point_count = output.size() / 2 - 1;
          for (int i = 0; i < point_count; i++) {
            mpc_x_vals.push_back(output[2+i]);
            mpc_y_vals.push_back(output[2+i+point_count]);
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;


          //.. add (x,y) points to list here, points are in reference to the
          // vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          next_x_vals.clear();
          next_y_vals.clear();
          for (double i = 0; i < 100; i+=10) {
            next_x_vals.push_back(i);
            double y = coeffs[0] + coeffs[1]*i + coeffs[2]*i*i + coeffs[3]*i*i*i;
            next_y_vals.push_back(y);
          }
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          //std::cout << "sleeping for " << actuator_delay << "ms" << std::endl;
          this_thread::sleep_for(chrono::milliseconds(actuator_delay));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
