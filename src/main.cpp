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
#include "planner.h"

using namespace std;
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

    // Path planner.
    auto planner = Planner(map);

    h.onMessage([&planner](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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

        // j[1] is the data JSON object
        auto output = planner.planPath(EnvContext(j));

        // Send the control data bak to the simulator.
        json msgJson;
        msgJson["next_x"] = output[0];
        msgJson["next_y"] = output[1];
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