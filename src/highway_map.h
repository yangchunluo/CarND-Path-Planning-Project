#pragma once

#include <string>
#include <vector>

class HighwayMap {
private:
    std::vector<double> waypoints_x;
    std::vector<double> waypoints_y;
    std::vector<double> waypoints_s;
    std::vector<double> waypoints_dx;
    std::vector<double> waypoints_dy;

public:
    void loadMap(const std::string &dataFile);
    std::vector<double> xyToFrenet(double x, double y, double theta);
    std::vector<double> frenetToXY(double s, double d);
    int getClosestWaypoint(double x, double y);
    int getNextWaypoint(double x, double y, double theta);
};
