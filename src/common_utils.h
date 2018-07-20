#pragma once

#include <cmath>

// For converting back and forth between radians and degrees.
inline constexpr double pi() { return 3.1415926; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }

inline double square(double x) { return x * x; }

/**
 * Converts back and forth between miles per hour to meters per second.
 */
inline double mph_to_mps(double mph) {
    return mph * 1609.344 /*meters per mile*/ / 3600 /*sec per hour*/;
}
inline double mps_to_mph(double mps) {
    return mps / mph_to_mps(1.0);
}

/**
 * Gets the lane center's offset.
 * @param lane
 * @param lane_width
 * @return lane center's d value
 */
inline double get_lane_center(int lane, int lane_width) {
    return lane_width / 2.0 + lane_width * lane;
}

/**
 * Gets the lane number.
 * @param d
 * @param lane_width
 * @return lane number
 */
inline int get_lane_number(double d, int lane_width) {
    return (int)(d / lane_width);
}

/**
 * Computes the Cartesian distance.
 */
inline double distance(double x1, double y1, double x2, double y2) {
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}
