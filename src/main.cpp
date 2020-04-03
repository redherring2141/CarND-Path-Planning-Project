#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"

#include <math.h>
#include <thread>
#include "spline.h"

#define VEL_MAX 49.8
#define ACC_MAX 0.25

// for convenience
using namespace std;
using nlohmann::json;
using std::string;
using std::vector;

int main()
{
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

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line))
  {
    istringstream iss(line);
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

  int ego_lane = 1;          // Ego car's lane
  double velocity_ego = 0.0;// Reference velocity in mph

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &ego_lane, &velocity_ego, // Added for better programming
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode)
  {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(data);

      if (s != "")
      {
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

          int size_pathx_prev = previous_path_x.size();

          if (size_pathx_prev>0)
          {// Prevent collision
            car_s = end_path_s;
          }

          // Prediction for near cars position and velocity
          bool is_car_ahead = false;
          bool is_car_left = false;
          bool is_car_right = false;

          // Identify the current lane line of near cars
          for (int idx_car=0; idx_car<sensor_fusion.size(); idx_car++)
          {
            float d_near = sensor_fusion[idx_car][6];
            int lane_near = -1;
            if      (d_near>0 && d_near<4) lane_near = 0;
            else if (d_near>4 && d_near<8) lane_near = 1;
            else if (d_near>8 && d_near<12)lane_near = 2;
            //else                            continue;
            if (lane_near < 0) continue;

            // Identify the surrounding car velocity
            double vx_near = sensor_fusion[idx_car][3];
            double vy_near = sensor_fusion[idx_car][4];
            double speed_near = sqrt(vx_near*vx_near + vy_near*vy_near);
            double s_near = sensor_fusion[idx_car][5];
            double roi_s_near = 25.0;
            double roi_offset = 15.0;

            // Predict the position of the near car in 0.02 seconds
            s_near += ((double)size_pathx_prev*speed_near*0.02);

            // Set the road ROI
            if (lane_near == ego_lane)
              is_car_ahead |= s_near > car_s && s_near - car_s <= roi_s_near;
            else if (lane_near - ego_lane == -1)
              is_car_left  |= car_s - roi_s_near + roi_offset <= s_near && car_s + roi_s_near >= s_near;
            else if (lane_near - ego_lane == 1)
              is_car_right |= car_s - roi_s_near + roi_offset <= s_near && car_s + roi_s_near >= s_near;
          }

          std::cout << "is_car_ahead " << is_car_ahead << "\t is_car_left " << is_car_left << "\t is_car_right " << is_car_right << std::endl;

          // Behavior planning
          double velocity_diff = 0.0;

          if (is_car_ahead)
          {
            if ( !is_car_left && ego_lane>0 )
              ego_lane = ego_lane-1;
            else if ( !is_car_right && ego_lane!=2 )
              ego_lane = ego_lane+1;
            else
              velocity_diff = -ACC_MAX;
          }
          else
          {
            if ( ego_lane!=1 )
              if ( ( ego_lane==0 && !is_car_right ) || ( ego_lane==2 && !is_car_left ) )
                ego_lane=1;
            if ( velocity_ego < VEL_MAX)
              velocity_diff = ACC_MAX;
          }
          
          vector<double> xpts_list;
          vector<double> ypts_list;

          double x_curr = car_x;
          double y_curr = car_y;
          double yaw_curr = deg2rad(car_yaw);

          if (size_pathx_prev < 4)
          {// Set the start position as the car position if the previous size is almost empty
            double x_prev = car_x - cos(car_yaw);
            double y_prev = car_y - sin(car_yaw);

            xpts_list.push_back(x_prev);
            xpts_list.push_back(car_x);
            ypts_list.push_back(y_prev);
            ypts_list.push_back(car_y);
          }
          else
          {// Set the start position as the end of the previous path
            x_curr = previous_path_x[size_pathx_prev - 1];
            y_curr = previous_path_y[size_pathx_prev - 1];

            double x_curr_prev = previous_path_x[size_pathx_prev-2];
            double y_curr_prev = previous_path_y[size_pathx_prev-2];

            yaw_curr = atan2(y_curr-y_curr_prev, x_curr-x_curr_prev);

            xpts_list.push_back(x_curr_prev);
            xpts_list.push_back(x_curr);
            ypts_list.push_back(y_curr_prev);
            ypts_list.push_back(y_curr);
          }


          // Add evenly 30 meters spaced points at the start point calculated above
          vector<double> waypt_0 = getXY(car_s+30, 2+4*ego_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> waypt_1 = getXY(car_s+60, 2+4*ego_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> waypt_2 = getXY(car_s+90, 2+4*ego_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

          // Add far points for fitting spline
          xpts_list.push_back(waypt_0[0]);
          xpts_list.push_back(waypt_1[0]);
          xpts_list.push_back(waypt_2[0]);

          ypts_list.push_back(waypt_0[1]);
          ypts_list.push_back(waypt_1[1]);
          ypts_list.push_back(waypt_2[1]);

          // Convert to the local car coordinates
          for (int idx_pts=0; idx_pts<xpts_list.size(); idx_pts++)
          {
            double xshift = xpts_list[idx_pts] - x_curr;
            double yshift = ypts_list[idx_pts] - y_curr;
            xpts_list[idx_pts] = xshift * cos(0 - yaw_curr) - yshift * sin(0 - yaw_curr);
            ypts_list[idx_pts] = xshift * sin(0 - yaw_curr) + yshift * cos(0 - yaw_curr);
          }


          // Create the spline object and fit (x,y) points to the spline
          tk::spline trajectory;
          trajectory.set_points(xpts_list, ypts_list);

          // Start with the previous path points from last line
          for (int idx_pts=0; idx_pts<size_pathx_prev; idx_pts++)
          {
            next_x_vals.push_back(previous_path_x[idx_pts]);
            next_y_vals.push_back(previous_path_y[idx_pts]);
          }

          // Break up the spline points
          double x_goal = 30.0;
          double y_goal = trajectory(x_goal);
          double dist_goal = sqrt(x_goal*x_goal + y_goal*y_goal);
          double x_interval = 0;

          
          for (int i=0; i<=50-size_pathx_prev; i++)
          {// Limit the jerk
            velocity_ego += velocity_diff;
            if (velocity_ego >= VEL_MAX)
              velocity_ego = VEL_MAX - 0.1;
            else if (velocity_ego <= ACC_MAX)
              velocity_ego += ACC_MAX;

            // Complete the path planning by filling up the remaining points
            double N = dist_goal / (0.02*velocity_ego/2.24);
            double x_pt = x_interval + x_goal/N;
            double y_pt = trajectory(x_pt);
            x_interval = x_pt;
            double x_pt_ref = x_pt;
            double y_pt_ref = y_pt;

            // Rotate
            x_pt = x_pt_ref * cos(yaw_curr) - y_pt_ref * sin(yaw_curr);
            y_pt = x_pt_ref * sin(yaw_curr) + y_pt_ref * cos(yaw_curr);
            // Translate
            x_pt += x_curr;
            y_pt += y_curr;

            next_x_vals.push_back(x_pt);
            next_y_vals.push_back(y_pt);
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