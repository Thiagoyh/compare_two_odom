
#pragma once

#include <vector>

#include "config/lua_parameter_dictionary.h"
#include "config/port.h"

namespace common {

// A 'FileResolver' for the 'LuaParameterDictionary' that reads files from disk.
// It searches the 'configuration_files_directories' in order to find the
// requested filename. The last place searched is always the
// 'configuration_files/' directory installed with Cartographer. It contains
// reasonable configuration for the various Cartographer components which
// provide a good starting ground for new platforms.
class ConfigurationFileResolver : public FileResolver {
 public:

  // c++11: explicit关键字 的作用就是防止类构造函数的隐式自动转换
  explicit ConfigurationFileResolver(
      const std::vector<std::string>& configuration_files_directories);

  std::string GetFullPathOrDie(const std::string& basename) override;
  std::string GetFileContentOrDie(const std::string& basename) override;

 private:
  std::vector<std::string> configuration_files_directories_;
};

}  // namespace common



