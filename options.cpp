#include "options.h"

#include <vector>
#include <memory>
#include "absl/memory/memory.h"
#include "config/configuration_file_resolver.h"
#include "glog/logging.h"

namespace common
{

    NodeOptions CreateNodeOptions(
    common::LuaParameterDictionary* const
        lua_parameter_dictionary) {

        NodeOptions node_options;
        LidarOptions lidar_param;
        OdomOptions odom_options;
        lidar_param.max_distance =
            lua_parameter_dictionary->GetDouble("max_distance");
        lidar_param.min_distance =
                lua_parameter_dictionary->GetDouble("min_distance");
        lidar_param.num_lines =
                lua_parameter_dictionary->GetInt("scan_lines");
        lidar_param.scan_period =
            lua_parameter_dictionary->GetDouble("scan_period");
        lidar_param.vertical_angle =
            lua_parameter_dictionary->GetDouble("vertical_angle");

        node_options.lidar_param_ = lidar_param;
        odom_options.map_resolution =
             lua_parameter_dictionary->GetDouble("map_resolution");
        odom_options.num_range_data =
            lua_parameter_dictionary->GetInt("num_range_data");
        odom_options.num_thread_pool =
            lua_parameter_dictionary->GetInt("num_thread_pool");
        odom_options.speed_filter =
            lua_parameter_dictionary->GetDouble("speed_filter");
        odom_options.scan_period =
                                 lidar_param.scan_period;

        return NodeOptions{lidar_param, odom_options};
    }

    NodeOptions LoadOptions(const std::string &configuration_directory,
                        const std::string &configuration_basename){
      auto file_resolver =
      absl::make_unique<common::ConfigurationFileResolver>(
          std::vector<std::string>{configuration_directory});

    const std::string code =
      file_resolver->GetFileContentOrDie(configuration_basename);

      common::LuaParameterDictionary lua_parameter_dictionary(
      code, std::move(file_resolver));

      return CreateNodeOptions(&lua_parameter_dictionary);
}

} // namespace common
