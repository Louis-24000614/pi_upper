/// @file
/// 见 math/angle.h。

#include "math/angle.h"

#include <cmath>

namespace servo {

int64_t PeriodNs(int period_us) { return static_cast<int64_t>(period_us) * kUsToNs; }

bool AngleToPulseNs(double angle_deg, int64_t* out_pulse_ns, int min_pulse_us, int max_pulse_us) {
  if (out_pulse_ns == nullptr) {
    return false;
  }
  if (angle_deg < kAngleMinDeg || angle_deg > kAngleMaxDeg) {
    return false;
  }
  const double span = static_cast<double>(max_pulse_us - min_pulse_us);
  const double pulse_us = min_pulse_us + span * (angle_deg / kAngleMaxDeg);
  *out_pulse_ns = static_cast<int64_t>(std::llround(pulse_us * kUsToNs));
  return true;
}

}  // namespace servo
