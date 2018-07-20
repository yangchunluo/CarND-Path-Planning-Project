#pragma once

#include <vector>

#include "common_utils.h"
#include "env_context.h"
#include "highway_map.h"
#include "json.hpp"

struct LaneInfo {
    struct {
        int car_id = -1;
        double speed = -1;
        double clearance = -1;
    } front, rear;
};

class Planner {
private:
    // Finite state machine for motion planning.
    enum {
        LANE_KEEPING,
        LANE_CHANGING
    } fsm = LANE_KEEPING;

    // Highway map.
    const HighwayMap& map;

    // Starting states are set by the simulator.
    double velocity = 0;
    int target_lane = 1;

    double planMotion(const EnvContext &context, const std::vector<LaneInfo> &lane_infos);
    void updateVelocity(const EnvContext &context, double target_speed);
    std::vector<std::vector<double>> generateTrajectory(const EnvContext &context) const;

public:
    explicit Planner(const HighwayMap &map): map(map) {}
    std::vector<std::vector<double>> planPath(const EnvContext &context);
};
