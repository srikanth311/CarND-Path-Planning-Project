#include "spline.h"
#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

double increment_velocity = 0.224 / 2;    // 0.05 m/s
int my_car_lane = 1;
double my_car_velocity = 0.0;

int findOtherCarLane(double d)
{
    if(d >=0 && d < 4)
        return 0;
    else if(d >= 4 && d < 8)
        return 1;
    else
        return 2;
}

bool isItSafeDistance(double main_car_s, double ot_car_s)
{
    if((ot_car_s > main_car_s) && (ot_car_s - main_car_s) < 30)
        return false;
    return true;
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */


          int previous_size = previous_path_x.size();
          if(previous_size > 0) {
              car_s = end_path_s;
          }

          bool is_left_lane_free = my_car_lane != 0;
          bool is_right_lane_free = my_car_lane != 2;
          bool is_front_car_closeby = false;

          double my_car_target_velocity = my_car_velocity;

          // Rotate through cars and gets front car closeness, find if the left lane is free and right lane is free?
          for(int car =0; car < sensor_fusion.size(); car++) {
              float d = sensor_fusion[car][6];
              double vx = sensor_fusion[car][3];
              double vy = sensor_fusion[car][4];
              double other_car_speed = sqrt(vx * vx + vy * vy);
              double other_car_s = sensor_fusion[car][5];
              int other_car_lane = findOtherCarLane(d);

              other_car_s = other_car_s + ((double)previous_size * 0.02 * other_car_speed);

              // Checking if my car lane and other car lane are same.
              if( my_car_lane == other_car_lane)
              {
                  // Checking if the other car is close by?
                  if(!isItSafeDistance(car_s, other_car_s))
                  {
                      my_car_target_velocity = other_car_speed * 2.24; // from miles per second to MPH.
                      is_front_car_closeby = true;
                  }
              }
              // Checking if other car is left side
              else if (other_car_lane == (my_car_lane - 1))
              {
                  // Checking if the other car is close by?
                  if(!isItSafeDistance(car_s - 5, other_car_s))
                  {
                      is_left_lane_free = false;
                  }
              }
              // Checking if other car is right side
              else if (other_car_lane == (my_car_lane + 1))
              {
                  // Checking if the other car is close by?
                  if(!isItSafeDistance(car_s - 5, other_car_s))
                  {
                      is_right_lane_free = false;
                  }
              }
          }

          // Checking if the front car is close? If it is close, need to match our car's velocity and change the lane,
          // If it is not close, maintain the max lane speed

          if ( is_front_car_closeby && (my_car_velocity >= my_car_target_velocity) )
          {
              my_car_velocity -= increment_velocity;
              if(is_left_lane_free)
                  my_car_lane--;
              else if(is_right_lane_free)
                  my_car_lane++;
          }
          else if (my_car_velocity < 49.5)
          {
              my_car_velocity += increment_velocity;
          }

          // Way points
          vector<double> x_points;
          vector<double> y_points;

          double x_reference = car_x;
          double y_reference = car_y;
          double yaw_reference = deg2rad(car_yaw);

          if(previous_size < 2) // If previous data points are not enough use our car data points as a starting points.
          {
              double previous_car_x = car_x - cos(car_yaw);
              double previous_car_y = car_y - sin(car_yaw);

              x_points.push_back(previous_car_x);
              x_points.push_back(car_x);

              y_points.push_back(previous_car_y);
              y_points.push_back(car_y);
          }
          else
          {
              x_reference = previous_path_x[previous_size - 1];
              y_reference = previous_path_y[previous_size - 1];

              double previous_reference_x = previous_path_x[previous_size - 2];
              double previous_reference_y = previous_path_y[previous_size - 2];
              yaw_reference = atan2(y_reference - previous_reference_y, x_reference - previous_reference_x);

              // use two points that make the path tangent to the previous path's end point
              x_points.push_back(previous_reference_x);
              x_points.push_back(x_reference);

              y_points.push_back(previous_reference_y);
              y_points.push_back(y_reference);
          }

          // Adding 30m spaced points.
          vector<double> next_waypoints_1 = getXY(car_s + 30, (2 + 4 * my_car_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_waypoints_2 = getXY(car_s + 60, (2 + 4 * my_car_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_waypoints_3 = getXY(car_s + 90, (2 + 4 * my_car_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          x_points.push_back(next_waypoints_1[0]);
          x_points.push_back(next_waypoints_2[0]);
          x_points.push_back(next_waypoints_3[0]);

          y_points.push_back(next_waypoints_1[1]);
          y_points.push_back(next_waypoints_2[1]);
          y_points.push_back(next_waypoints_3[1]);

          for (int i = 0; i < x_points.size(); i++)
          {
                double shift_x = x_points[i] - x_reference;
                double shift_y = y_points[i] - y_reference;

                x_points[i] = shift_x * cos(0 - yaw_reference) - shift_y * sin(0 - yaw_reference);
                y_points[i] = shift_x * sin(0 - yaw_reference) + shift_y * cos(0 - yaw_reference);
          }

          tk::spline sp_line;
          sp_line.set_points(x_points, y_points);

          // Staring with previous path points
          for (int i = 0; i < previous_size; i++)
          {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
          }

          // Break up the spline points so that we go at our refence velocity.
          double x_target = 30;
          double y_target = sp_line(x_target);
          double distance_target = sqrt(x_target * x_target + y_target * y_target);

          double x_add_on = 0;

          for (int i = 1; i < 50 - previous_size; i++)
          {

                double N = distance_target / (0.02 * my_car_velocity / 2.24);
                double x_point = x_add_on + x_target / N;
                double y_point = sp_line(x_point);

                x_add_on = x_point;

                double x_ref = x_point;
                double y_ref = y_point;

                // rotate back to normal
                x_point = x_ref * cos(yaw_reference) - y_ref * sin(yaw_reference);
                y_point = x_ref * sin(yaw_reference) + y_ref * cos(yaw_reference);

                x_point += x_reference;
                y_point += y_reference;

                next_x_vals.push_back(x_point);
                next_y_vals.push_back(y_point);
          }


          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
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