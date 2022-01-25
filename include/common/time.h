#pragma once

#include <chrono>

namespace common
{
    struct UniversalTimeScaleClock {
    using rep = int64_t;
    using period = std::ratio<1, 10000000>;
    using duration = std::chrono::duration<rep, period>;
    using time_point = std::chrono::time_point<UniversalTimeScaleClock>;
    static constexpr bool is_steady = true;
};
    using Duration = UniversalTimeScaleClock::duration;

    Duration FromSeconds(const double seconds);

} // namespace common
