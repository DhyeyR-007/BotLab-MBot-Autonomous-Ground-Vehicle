#include <utils/lcm_config.h>
#include <mbot/mbot_channels.h>
#include <mbot_lcm_msgs/path2D_t.hpp>
#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <unistd.h>
#include <cmath>
#include <chrono>
#include <vector>
#include <algorithm>
using namespace mbot_lcm_msgs;
int main(int argc, char** argv)
{
    float a = 0.61;
    std::vector<pose2D_t> mazeWaypoints = {
        {0, 0.0f, 0.0f, 0.0f},  // Start position
        {0, a, 0.0f, 0.0f},
        {0, a, -a, 0.0f},
        {0, 2*a, -a, 0.0f},
        {0, 2*a, a, 0.0f},
        {0, 3*a, a, 0.0f},
        {0, 3*a, -a, 0.0f},
        {0, 4*a, -a, 0.0f},
        {0, 4*a, 0.0f, 0.0f},
        {0, 5*a, 0.0f, 0.0f},
    };

    std::cout << "Commanding robot to navigate the maze.\n";

    path2D_t path;
    path.path = mazeWaypoints;
    path.path_length = path.path.size();

    auto now = std::chrono::system_clock::now();
    auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
    auto epoch = now_ms.time_since_epoch();
    auto value = std::chrono::duration_cast<std::chrono::milliseconds>(epoch);
    path.utime = value.count();
    lcm::LCM lcmInstance(MULTICAST_URL);
    std::cout << "publish to: " << CONTROLLER_PATH_CHANNEL << std::endl;
    lcmInstance.publish(CONTROLLER_PATH_CHANNEL, &path);
    sleep(1);

    return 0;
}