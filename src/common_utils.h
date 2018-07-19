#pragma once

// For converting back and forth between radians and degrees.
inline constexpr double pi() { return 3.1415926; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }

/**
 * Converts miles per hour to meters per second.
 */
inline double mph_to_mps(double mph) {
    return mph * 1609.344 /*meters per mile*/ / 3600 /*sec per hour*/;
}

/**
 * Gets the lane center's offset.
 * @param lane
 * @param lane_width
 */
inline double get_lane_center(int lane, int lane_width) {
    return lane_width / 2.0 + lane_width * lane;
}

/**
 * Computes the Cartesian distance.
 */
inline double distance(double x1, double y1, double x2, double y2) {
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}
