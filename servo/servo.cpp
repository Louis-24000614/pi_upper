/// @file
/// 见 servo.h。

#include "servo.h"

namespace servo {

Servo::Servo(SysfsPwm* pwm, int period_us, int min_pulse_us, int max_pulse_us)
    : pwm_(pwm), period_us_(period_us), min_pulse_us_(min_pulse_us), max_pulse_us_(max_pulse_us) {}

bool Servo::SetAngle(double angle_deg) {
  if (pwm_ == nullptr) {
    return false;
  }
  int64_t duty = 0;
  if (!AngleToPulseNs(angle_deg, &duty, min_pulse_us_, max_pulse_us_)) {
    return false;
  }
  return pwm_->Configure(PeriodNs(period_us_), duty);
}

bool Servo::Close() {
  if (pwm_ == nullptr) {
    return true;
  }
  return pwm_->Disable();
}

}  // namespace servo
