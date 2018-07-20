#include <exception>
#include <cstdio>

#include "planner.h"
#include "spline.h"

#include <iostream>
#include <fstream>

using namespace std;

static int get_lane_from_file() {
    ifstream control_file("/tmp/control.txt", ifstream::in);
    if (!control_file.is_open()) {
        return -1;
    }
    string line;
    getline(control_file, line);
    istringstream iss(line);
    int lane;
    iss >> lane;
    return lane;
}

vector<vector<double>> Planner::planPath(const EnvContext &context) {
    const auto prev_size = context.previous_path_x.size();
    const double car_s = prev_size > 0 ? context.end_path_s : context.car_s;

    // 1. Determine the lane and velocity.
    double target_speed;


//    int target_lane = get_lane_from_file();
//    if (target_lane >= 0 && target_lane != lane) {
//        cout << "switching from lane " << lane << " to " << target_lane << endl;
//        lane = target_lane;
//    }

    switch (fsm) {
        case LANE_KEEPING: {
            // Get the distance and speed of the closest car in front of us.
            double closest_car_id = -1, closest_distance = -1, closest_speed = -1;
            for (auto check_car : context.sensor_fusion) {
                auto id = (int)check_car[0];
                auto d = check_car[6];
                if (get_lane_number(d, context.lane_width) != lane) {
                    // The car is not in my lane.
                    continue;
                }
                auto vx = check_car[3];
                auto vy = check_car[4];
                auto check_speed = sqrt(vx * vx + vy * vy);
                auto check_car_s = check_car[5];
                // Take the simulator delay into account to predict where the car is.
                check_car_s += prev_size * context.sim_update_freq * check_speed;
                if (check_car_s < car_s || check_car_s - car_s > context.safety_margin) {
                    // The car is either behind or too far ahead.
                    continue;
                }
                if (closest_distance > 0 && check_car_s - car_s > closest_distance) {
                    // Not the closest car.
                    continue;
                }
                closest_car_id = id;
                closest_distance = check_car_s - car_s;
                closest_speed = check_speed;
            }
            if (closest_car_id < 0) {
                // No car is in the safe distance in front of our lane.
                target_speed = mph_to_mps(context.speed_limit_mph);
                 printf("No car detected in front, current speed %.2f m/s, target speed %.2f m/s\n",
                       velocity, target_speed);
            } else {
                // Decrease the speed to the lane speed.
                printf("Car %d detected in %.2f meters, current speed %.2f m/s, lane speed %.2f m/s\n",
                       (int)closest_car_id, closest_distance, velocity, closest_speed);

                // Check if we have enough break distance.
                if (velocity < closest_speed ||  // We will never catch up.
                    closest_distance > 0.5 * square(velocity - closest_speed) / context.acc_limit) {
                    target_speed = closest_speed;
                } else {
                    printf("!!!!!!Not enough brake distance!!!!!!\n");
                    target_speed = closest_speed;
                }
            }

        } break;

        default:
            throw invalid_argument("Invalid state");
    }

    // Adjust speed.
    updateVelocity(context, target_speed);

    // 2. Generate trajectory.
    return generateTrajectory(context);

}

/**
 * Updates velocity based on the target speed.
 * @param context
 * @param target_speed
 */
void Planner::updateVelocity(const EnvContext &context, double target_speed) {
    if (abs(velocity - target_speed) < context.acc_limit * context.sim_update_freq) {
        // Can reach the target speed in this update frame.
        velocity = target_speed;
    } else {
        // Gradually reach the target speed so that we don't exceed max jerk limit.
        velocity += context.acc_limit * context.sim_update_freq
                    * (velocity < target_speed ? 1.0 : -1.0);
    }
}

/**
 * Generates smooth and jerk-minimizing trajectory using spline.
 * @param context
 * @return trajectory in x, y global coordinates.
 */
vector<vector<double>> Planner::generateTrajectory(const EnvContext &context) const {
    const auto prev_size = context.previous_path_x.size();
    const double car_s = prev_size > 0 ? context.end_path_s : context.car_s;

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
