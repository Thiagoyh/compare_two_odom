#pragma once

#include <string>
#include "config/lua_parameter_dictionary.h"
#include "config/port.h"

namespace common{

    struct LidarOptions{
        double max_distance;
        double min_distance;
        int num_lines;
        double scan_period;
        int points_per_line;
        double horizontal_angle_resolution;
        double horizontal_angle;
        double vertical_angle_resolution;
        double vertical_angle;
    };
    struct OdomOptions{
            double map_resolution = 0.4;
            int num_range_data = 30;

            int num_thread_pool = 4;
            double speed_filter = 0.1;
            double scan_period = 0.1;
    };

    class NodeOptions{
        public:
            LidarOptions lidar_param_;
            OdomOptions odom_options;
    };

    NodeOptions CreateNodeOptions(
    common::LuaParameterDictionary* lua_parameter_dictionary);

    NodeOptions LoadOptions(const std::string &configuration_directory,
                        const std::string &configuration_basename);
}