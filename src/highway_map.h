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
    std::vector<double> xyToFrenet(double x, double y, double theta) const;
    std::vector<double> frenetToXY(double s, double d) const;
    int getClosestWaypoint(double x, double y) const;
    int getNextWaypoint(double x, double y, double theta) const;
};
