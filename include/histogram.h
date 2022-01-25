#pragma once

#include <vector>
#include <string>

#include "glog/logging.h"
#include "config/port.h"


namespace common {

class Histogram {
 public:
  void Add(float value);
  std::string ToString(int buckets) const;

 private:
  std::vector<float> values_;
};

}  // namespace common