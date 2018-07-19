#include "planner.h"
#include "spline.h"

#include <iostream>

using namespace std;

std::vector<std::vector<double>> Planner::planPath(const EnvContext &context) {
    auto prev_size = context.previous_path_x.size();
    double car_s = prev_size > 0 ? context.end_path_s : context.car_s;

    // 1. Determine the lane and velocity.

    // Check if we are too close to the car in front.
    bool should_slow_down = false;
    for (auto check_car : context.sensor_fusion) {
        auto d = check_car[6];
        if (d > context.lane_width * lane && d < context.lane_width * (lane + 1)) {
            // This car is in our lane.
            auto vx = check_car[3];
            auto vy = check_car[4];
            double check_speed = sqrt(vx * vx + vy * vy);
            double check_car_s = check_car[5];
            // Take the simulator delay into account to predict the future.
            check_car_s += prev_size * context.sim_update_freq * check_speed;
            if (check_car_s > car_s && check_car_s - car_s < context.safety_margin) {
                should_slow_down = true;
            }
        }
    }

    // Adjust car speed.
    if (should_slow_down) {
        velocity -= context.acc_limit;
    } else if (velocity < mph_to_mps(context.speed_limit_mph)) {
        velocity += context.acc_limit;
    }


    // 2. Generate trajectory.
    vector<double> pts_x, pts_y;

    double ref_x;
    double ref_y;
    double ref_yaw;

    if (prev_size < 2) {
        // Previous size is almost empty, use car pose as the starting reference.
        ref_x = context.car_x;
        ref_y = context.car_y;
        ref_yaw = deg2rad(context.car_yaw);

        // Just use two points that make the path tangent to the car.
        pts_x.push_back(context.car_x - cos(context.car_yaw));
        pts_x.push_back(context.car_x);
        pts_y.push_back(context.car_y - sin(context.car_yaw));
        pts_y.push_back(context.car_y);
    } else {
        // Use the previous path as the starting reference.
        ref_x = context.previous_path_x[prev_size - 1];
        ref_y = context.previous_path_y[prev_size - 1];

        double prev_ref_x = context.previous_path_x[prev_size - 2];
        double prev_ref_y = context.previous_path_y[prev_size - 2];
        ref_yaw = atan2(ref_y - prev_ref_y, ref_x - prev_ref_x);

        // Use two points that make the path tangent to the previous path's end point.
        pts_x.push_back(prev_ref_x);
        pts_x.push_back(ref_x);
        pts_y.push_back(prev_ref_y);
        pts_y.push_back(ref_y);
    }

    // In Frenet space, add a few evenly 30m spaced points ahead of the starting reference.
    for (int i = 0; i < 3; i++) {
        auto xy = map.frenetToXY(car_s + 30 * (i + 1), get_lane_center(lane, context.lane_width));
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
        next_x_vals.push_back(context.previous_path_x[i]);
        next_y_vals.push_back(context.previous_path_y[i]);
    }

    // Set up the horizon.
    double horizon_x = 30.0;
    double horizon_y = spline(horizon_x);
    double horizon_dist = distance(horizon_x, horizon_y, 0, 0);
    // The spacing of the points depends on reference speed and simulator update frequency.
    double N = horizon_dist / (context.sim_update_freq * velocity);

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
    return {next_x_vals, next_y_vals};
}