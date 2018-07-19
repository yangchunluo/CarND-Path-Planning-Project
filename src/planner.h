#pragma once

#include <vector>

#include "common_utils.h"
#include "env_context.h"
#include "highway_map.h"
#include "json.hpp"

class Planner {
private:
    // Map
    const HighwayMap& map;

    // Starting conditions.
    double velocity = 0;
    int lane = 1;

public:
    explicit Planner(const HighwayMap &map): map(map) {}
    std::vector<std::vector<double>> planPath(const EnvContext &context);
};
