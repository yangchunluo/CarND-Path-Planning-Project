#pragma once

#include <vector>

#include "common_utils.h"
#include "env_context.h"
#include "highway_map.h"
#include "json.hpp"

class Planner {
private:
    // Finite state machine for motion planning.
    enum {
        LANE_KEEPING,
        LANE_CHANGING
    } fsm = LANE_KEEPING;


    // Highway map.
    const HighwayMap& map;

    // Starting states (set by the simulator).
    double velocity = 0;
    int lane = 1;

    void updateVelocity(const EnvContext &context, double target_speed);
    std::vector<std::vector<double>> generateTrajectory(const EnvContext &context) const;

public:
    explicit Planner(const HighwayMap &map): map(map) {}
    std::vector<std::vector<double>> planPath(const EnvContext &context);
};
