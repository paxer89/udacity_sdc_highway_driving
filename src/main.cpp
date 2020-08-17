#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

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
  
  
  
  // int for start lane
  int lane = 1;
  
  //starting velocity
  double ref_vel = 0.0;
  
  

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &ref_vel, &lane]
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
          
          //points handed over to the simulater
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */                
          
          

          
          //value for longitudinal accel
          double comfort_acc = 0.3;          
          
          
          // size of "leftover" of previous path
          int prev_size = previous_path_x.size();
          
          // predicted s of ego car at end of previous path
          double car_s_pred;
          
          if(prev_size>0){            
           car_s_pred = end_path_s; 
          }
          else{
            car_s_pred = car_s;
          }
          
          // bool value if car in front in ego lane is too close
          bool too_close = false;
          // bool value if lane left of ego car is free for lane change
          bool is_free_left = true;
          // bool value if lane right of ego car is free for lane change
          bool is_free_right = true;
          
          //double front_speed_ego;
          //double front_speed_left;
          //double front_speed_right;
          
          
          // check all objects in sensor_fusion check if there is an object too close in front of ego
          for(int i=0; i<sensor_fusion.size(); i++){            
            
            float d = sensor_fusion[i][6];
            
            //if other car is in my lane
            if(d<(2+4*lane+2) && d>(2+4*lane-2)){
              
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx+vy*vy);
              double check_car_s = sensor_fusion[i][5];
              
              //predict car to my end of path time
              check_car_s += (double)prev_size*0.02*check_speed;
              
              if( (check_car_s > car_s_pred) && ((check_car_s-car_s_pred) < 30.0) ) {
              //if( (check_car_s-car_s_pred) < 30 ){
                
                too_close = true;
              }             
            }
            
            //if other car is in left lane
            if(d<(2+4*(lane-1)+2) && d>(2+4*(lane-1)-2)){
              
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx+vy*vy);
              double check_car_s = sensor_fusion[i][5];
              
              //predict car to my end of path time
              check_car_s += (double)prev_size*0.02*check_speed;
              
              if( (check_car_s > car_s_pred) && ((check_car_s-car_s_pred) < 30.0) ) {
              //if( (check_car_s-car_s_pred) < 30 ){
                
                is_free_left = false;
              }             
            }
            
            //if other car is in right lane
            if(d<(2+4*(lane+1)+2) && d>(2+4*(lane+1)-2)){
              
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx+vy*vy);
              double check_car_s = sensor_fusion[i][5];
              
              //predict car to my end of path time
              check_car_s += (double)prev_size*0.02*check_speed;
              
              if( (check_car_s > car_s_pred) && ((check_car_s-car_s_pred) < 30.0) ) {
              //if( (check_car_s-car_s_pred) < 30 ){
                
                is_free_right = false;
              }             
            }           
            
            
          }
          
          
          // check if there is one more lane on the left
          if ((lane-1)<0){
            is_free_left = false;
            //std::cout << "false";          
          }
          
          // check if there is one more lane on the left
          if ((lane+1)>2){
            is_free_right = false;
            //std::cout << "false";          
          }
          
          
          // if a car is too close in ego lane
          if(too_close){
            //decallerate
            ref_vel -= comfort_acc;
            
            //change lane left
            if (is_free_left){
              lane--;
            }
            //chnage lane right
            else if (is_free_right){
              lane++;
            }
          }
          //accelerate
          else if(ref_vel < 49.5){
            ref_vel += comfort_acc;            
          }
          

          
          
          //anchor points for spline
          vector<double> anchor_pts_x;
          vector<double> anchor_pts_y;  
          
          //current state of the car          
          double ref_x = car_x;
          double ref_y = car_y;
          
          //double ref_yaw = car_yaw;
          double ref_yaw = deg2rad(car_yaw);
          
          //if previous path is not long enough use the car state as starting position
          if(prev_size < 2){
            
            //tangent to the car
            double prev_car_x = car_x - cos(ref_yaw);
            double prev_car_y = car_y - sin(ref_yaw);
            
            anchor_pts_x.push_back(prev_car_x);
            anchor_pts_x.push_back(car_x);
            
            anchor_pts_y.push_back(prev_car_y);
            anchor_pts_y.push_back(car_y);             
          }
          else {
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];
            
            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);
            
            anchor_pts_x.push_back(ref_x_prev);
            anchor_pts_x.push_back(ref_x);
            
            anchor_pts_y.push_back(ref_y_prev);
            anchor_pts_y.push_back(ref_y);   
          }
          
          
          
          // add 3 more anchorpoints in 30m distance to the anchor points for spline
          vector<double> next_wp0 = getXY(car_s+50,(2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+70,(2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90,(2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          anchor_pts_x.push_back(next_wp0[0]);
          anchor_pts_x.push_back(next_wp1[0]);
          anchor_pts_x.push_back(next_wp2[0]);
          
          anchor_pts_y.push_back(next_wp0[1]);
          anchor_pts_y.push_back(next_wp1[1]);
          anchor_pts_y.push_back(next_wp2[1]);  
          
          
          for (int i=0; i<anchor_pts_x.size(); i++){
            
            //transform anchor points into car coordinates
            double shift_x = anchor_pts_x[i]-ref_x;
            double shift_y = anchor_pts_y[i]-ref_y;
            
            anchor_pts_x[i] = (shift_x *cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
            anchor_pts_y[i] = (shift_x *sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));             
          }
          
          //create a spline
          tk::spline s;
          
          //set anchor points for spline (spline in car coordinates)
          s.set_points(anchor_pts_x, anchor_pts_y);
          
          
          //add remaining points from previous path to new path
          for (int i = 0; i < previous_path_x.size(); ++i) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          //values for linearizing velocity
          //x and y in vehicle coordinates
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x+target_y*target_y);
          
          double x_add_on = 0;
          
          
          
          for (int i = 0; i < 50-previous_path_x.size(); ++i) {
            
            //number of points per target distance: target_dist/ update_rate*vel/2.24 mph per m/s
            double N = target_dist/(0.02*ref_vel/2.24);
            
            double x_point = x_add_on + target_x/N;
            double y_point = s(x_point);
            
            x_add_on = x_point;
            
            
            double x_temp = x_point;
            double y_temp = y_point;
            
            //rotate back to global coordinates
            x_point = x_temp*cos(ref_yaw)-y_temp*sin(ref_yaw);
            y_point = x_temp*sin(ref_yaw)+y_temp*cos(ref_yaw);
            
            //translate back in global coordinates
            x_point += ref_x;
            y_point += ref_y;
              
              
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
            
          }   
                    
          
          
          
          
          //END of TODO   
          
          
          
          

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