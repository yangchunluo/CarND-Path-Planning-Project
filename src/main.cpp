#include <chrono>
#include <cmath>
#include <limits>
#include <thread>
#include <uWS/uWS.h>
#include <vector>

#include "common_utils.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "highway_map.h"
#include "json.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(const string &s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of('[');
    auto b2 = s.find_first_of('}');
    if (found_null != string::npos) {
        return "";
    } else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

int main() {
    uWS::Hub h;

    // Static highway map.
    HighwayMap map;
    map.loadMap("../data/highway_map.csv");

    // Lane is [0, 2], from road center to right side of the road.
    int lane = 1;
    // speed in mph.
    double velocity = 0;

    h.onMessage([&lane, &velocity, &map]
                 (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        // auto sdata = string(data).substr(0, length);
        // cout << sdata << endl;
        if (!(length > 2 && data[0] == '4' && data[1] == '2')) {
            return;
        }

        auto s = hasData(data);
        if (s.empty()) {
            // Manual driving
            std::string msg = "42[\"manual\",{}]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            return;
        }

        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event != "telemetry") {
            return;
        }

        // Some constant values.
        // The max s value before wrapping around the track back to 0
        const double max_s = 6945.554;
        // Lane width in meters.
        const int lane_width = 4;
        // Simulator's frequency to update car pose (seconds).
        const double sim_update_freq = 0.02;
        // Speed limit in meters per second.
        const double speed_limit = mph_to_mps(49.5);
        // Acceleration/deceleration limit.
        const double acc_limit = 5 /*meters per second^2*/ * sim_update_freq;
        // Safety margin in meters.
        const double safety_margin = 30;

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
        // Sensor Fusion Data, a list of all other cars on the same side of the road.
        vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];

        auto prev_size = previous_path_x.size();

        // Check if we are too close to the car in front.
        if (prev_size > 0) {
            car_s = end_path_s;
        }

        bool should_slow_down = false;
        for (auto check_car : sensor_fusion) {
            auto d = check_car[6];
            if (d > lane_width * lane && d < lane_width * (lane + 1)) {
                // This car is in our lane.
                auto vx = check_car[3];
                auto vy = check_car[4];
                double check_speed = sqrt(vx * vx + vy * vy);
                double check_car_s = check_car[5];
                // Take the simulator delay into account to predict the future.
                check_car_s += prev_size * sim_update_freq * check_speed;
                if (check_car_s > car_s && check_car_s - car_s < safety_margin) {
                    should_slow_down = true;
                }
            }
        }

        // Adjust car speed.
        if (should_slow_down) {
            velocity -= acc_limit;
        } else if (velocity < speed_limit) {
            velocity += acc_limit;
        }

        // Trajectory generation based on lane and speed.
        vector<double> pts_x, pts_y;

        double ref_x;
        double ref_y;
        double ref_yaw;

        if (prev_size < 2) {
            // Previous size is almost empty, use car pose as the starting reference.
            ref_x = car_x;
            ref_y = car_y;
            ref_yaw = deg2rad(car_yaw);

            // Just use two points that make the path tangent to the car.
            pts_x.push_back(car_x - cos(car_yaw));
            pts_x.push_back(car_x);
            pts_y.push_back(car_y - sin(car_yaw));
            pts_y.push_back(car_y);
        } else {
            // Use the previous path as the starting reference.
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];

            double prev_ref_x = previous_path_x[prev_size - 2];
            double prev_ref_y = previous_path_y[prev_size - 2];
            ref_yaw = atan2(ref_y - prev_ref_y, ref_x - prev_ref_x);

            // Use two points that make the path tangent to the previous path's end point.
            pts_x.push_back(prev_ref_x);
            pts_x.push_back(ref_x);
            pts_y.push_back(prev_ref_y);
            pts_y.push_back(ref_y);
        }

        // In Frenet space, add a few evenly 30m spaced points ahead of the starting reference.
        for (int i = 0; i < 3; i++) {
            auto xy = map.frenetToXY(car_s + 30 * (i + 1), get_lane_center(lane, lane_width));
            pts_x.push_back(xy[0]);
            pts_y.push_back(xy[1]);
        }

        // Convert the points from global to car coordinate system (to make later math easier).
        for (int i = 0; i < pts_x.size(); i++) {
            double shift_x = pts_x[i] - ref_x;
            double shift_y = pts_y[i] - ref_y;
            pts_x[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
            pts_y[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
        }

        // Define the spline.
        tk::spline spline;
        spline.set_points(pts_x, pts_y);

        // Define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
        vector<double> next_x_vals;
        vector<double> next_y_vals;
        // Start with the previous way points.
        for (int i = 0; i < prev_size; i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
        }

        // Set up the horizon.
        double horizon_x = 30.0;
        double horizon_y = spline(horizon_x);
        double horizon_dist = distance(horizon_x, horizon_y, 0, 0);
        // The spacing of the points depends on reference speed and simulator update frequency.
        double N = horizon_dist / (sim_update_freq * velocity);

        double x_add_on = 0;
        for (int i = 1; i <= 50 - prev_size; i++) {
            double point_x = x_add_on + horizon_x / N;
            double point_y = spline(point_x);
            x_add_on = point_x;

            // Convert the point from car to global coordinate system.
            double shift_x = point_x;
            double shift_y = point_y;
            point_x = shift_x * cos(ref_yaw) - shift_y * sin(ref_yaw);
            point_y = shift_x * sin(ref_yaw) + shift_y * cos(ref_yaw);
            point_x += ref_x;
            point_y += ref_y;

            next_x_vals.push_back(point_x);
            next_y_vals.push_back(point_y);
        }

        // Send the control data bak to the simulator.
        json msgJson;
        msgJson["next_x"] = next_x_vals;
        msgJson["next_y"] = next_y_vals;
        auto msg = "42[\"control\","+ msgJson.dump()+"]";
        // this_thread::sleep_for(chrono::milliseconds(1000));
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
    });

    // We don't need this since we're not using HTTP but if it's removed the
    // program doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
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
