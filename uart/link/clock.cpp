#include "link/clock.h"

#include <chrono>

namespace uart {
namespace {

uint64_t SteadyNanos() {
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
}

}  // namespace

SteadyClock::SteadyClock() : origin_ns_(SteadyNanos()) {}

uint64_t SteadyClock::NowUs() const { return (SteadyNanos() - origin_ns_) / 1000; }

}  // namespace uart
