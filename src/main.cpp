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
#include "spline.h"



// for convenience
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

    std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

    string line;
    while (getline(in_map_, line)) 
    {
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

    int lane = 1; // start in lane 1
    double ref_vel = 0; // miles per hour
    double max_vel = 49.5;//dont want to keep it at 50, because that may cause slight overshoot
    double max_acceleration = 0.224;
    double safety_distance = 30;

        h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&lane,&ref_vel,
               &max_vel, &safety_distance, &max_acceleration]
                (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                uWS::OpCode opCode) 
    {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {

            auto s = hasData(data);

            if (s != "") 
            {
                auto j = json::parse(s);
                
                string event = j[0].get<string>();
                
                if (event == "telemetry") 
                {
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

                    vector<double> x_points;
                    vector<double> y_points;

                    /**
                    * TODO: define a path made up of (x,y) points that the car will visit
                    *   sequentially every .02 seconds
                    */
                    int size_of_previous_path = previous_path_x.size();

                    if(size_of_previous_path>0)
                    {
                        car_s = end_path_s;
                    }

                    //Using sensor fusion data to determine presence of other vehicles, to implement the state machine
                    // it is useful to know how many cars are there in lanes around us, and their position relative to us
                    //for the state machine, we also need our velocity, and the velocities of the vehicles around us
                    
                    bool left_lane = false;
                    bool right_lane = false;
                    bool our_lane = false;
                    

                    for(int i = 0; i< sensor_fusion.size(); i++)
                    {
                        //get the d value of the car
                        float d = sensor_fusion[i][6];
                        double vx = sensor_fusion[i][3];
                        double vy = sensor_fusion[i][4];
                        double get_car_speed = sqrt(vx*vx+vy*vy);//vector sum fo vx and vy
                        double get_car_s = sensor_fusion[i][5];
                        double get_car_lane = -1;
                        //car lane based on 'd' distance
                        if(d<=4.0)
                        {
                            get_car_lane = 0;
                        }
                        else if(d>4.0 && d<8.0)
                        {
                            get_car_lane = 1;
                        }
                        else
                        {
                            get_car_lane = 2;
                        }

                        // Estimate car s position after executing previous trajectory.
                        get_car_s += ((double)size_of_previous_path*0.02*get_car_speed);

                        //groundwork for state machine of state machine
                        if(get_car_lane == lane)
                        {
                            //if the car is in our lane, check if future position is viable or not
                            our_lane |= (get_car_s > car_s) && (get_car_s - car_s < safety_distance);
                        }else if ( get_car_lane - lane == -1 ) 
                        {
                            // Car is in the left lane w.r.t us
                            left_lane |= (car_s - safety_distance < get_car_s) && (car_s + safety_distance > get_car_s);
                        } else if ( get_car_lane - lane == 1 ) 
                        {
                            //car is in right lane w.r.t us
                            right_lane |= (car_s - safety_distance < get_car_s )&& (car_s + safety_distance > get_car_s);
                        }

                    }
                    //creating rough state machine based on our speed and occupance of lanes
                    if(our_lane)
                    {
                        //If our lane is occupied
                        if(lane > 0 && !left_lane)
                        {
                            //if we arent on leftmost lane, and no car on left lane, then switch to left lane, as we prioritise left overtake
                            lane--;
                        }
                        else if(lane < 2 && !right_lane)
                        {
                            //if we arent on rightmost lane, and no car on right lane, then switch to right lane
                            lane++;
                        }
                        else
                        {
                            //slow down car because no other alternative
                            ref_vel-=max_acceleration;
                            
                        }

                    } else
                    {
                        //our lane is free
                        //try switching to slow lane if possible
                        if(lane < 2 && !right_lane)
                        {
                            lane++;
                        }
                        if(ref_vel < max_vel)
                        {
                            ref_vel+=max_acceleration;
                        }
                        
                    }

                    double ref_x = car_x;
                    double ref_y = car_y;
                    double ref_yaw = deg2rad(car_yaw);

                    if(size_of_previous_path < 2)
                    {
                        double prev_car_x = car_x - cos(car_yaw);
                        double prev_car_y = car_y - sin(car_yaw);

                        x_points.push_back(prev_car_x);
                        x_points.push_back(car_x);

                        y_points.push_back(prev_car_y);
                        y_points.push_back(car_y);
                    }
                    else
                    {
                        ref_x = previous_path_x[size_of_previous_path - 1];
                        ref_y = previous_path_y[size_of_previous_path - 1];

                        double ref_x_prev = previous_path_x[size_of_previous_path - 2];
                        double ref_y_prev = previous_path_y[size_of_previous_path - 2];
                        ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

                        x_points.push_back(ref_x_prev);
                        x_points.push_back(ref_x);

                        y_points.push_back(ref_y_prev);
                        y_points.push_back(ref_y);
                    }
                    
                    // potential future points
                    vector<double> next_wp0 = getXY(car_s + safety_distance, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                    vector<double> next_wp1 = getXY(car_s + 2*safety_distance, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                    vector<double> next_wp2 = getXY(car_s + 3*safety_distance, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

                    x_points.push_back(next_wp0[0]);
                    x_points.push_back(next_wp1[0]);
                    x_points.push_back(next_wp2[0]);

                    y_points.push_back(next_wp0[1]);
                    y_points.push_back(next_wp1[1]);
                    y_points.push_back(next_wp2[1]);

                    //Converting to car cordinates
                    for ( int i = 0; i < x_points.size(); i++ ) 
                    {
                        double shift_x = x_points[i] - ref_x;
                        double shift_y = y_points[i] - ref_y;

                        x_points[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
                        y_points[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
                    }

                    // Create the spline.
                    tk::spline s;
                    s.set_points(x_points, y_points);

                    // Output path points from previous path for continuity.
                    for ( int i = 0; i < size_of_previous_path; i++ ) 
                    {
                        next_x_vals.push_back(previous_path_x[i]);
                        next_y_vals.push_back(previous_path_y[i]);
                    }
                    // Calculate distance y position on 30 m ahead.
                    double target_x = 30.0;
                    double target_y = s(target_x);
                    double target_dist = sqrt(target_x*target_x + target_y*target_y);

                    double x_add_on = 0;

                    for( int i = 1; i < 50 - size_of_previous_path; i++ ) 
                    {
                        
                        if ( ref_vel > max_vel ) 
                        {
                            ref_vel = max_vel;
                        } 
                        double N = target_dist/(0.02*ref_vel/2.24);
                        double x_point = x_add_on + target_x/N;
                        double y_point = s(x_point);

                        x_add_on = x_point;

                        double x_ref = x_point;
                        double y_ref = y_point;

                        x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
                        y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

                        x_point += ref_x;
                        y_point += ref_y;

                        next_x_vals.push_back(x_point);
                        next_y_vals.push_back(y_point);
                    }


                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;

                    auto msg = "42[\"control\","+ msgJson.dump()+"]";

                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }  // end "telemetry" if
            } else 
            {
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
    if (h.listen(port)) 
    {
        std::cout << "Listening to port " << port << std::endl;
    } else 
    {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }

    h.run();
}