#include "time.h"

namespace common{
    Duration FromSeconds(const double seconds) {
   return std::chrono::duration_cast<Duration>(
      std::chrono::duration<double>(seconds));
}
}