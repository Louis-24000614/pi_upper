/// @file
/// 舵机门面：角度 → 脉宽 → sysfs PWM。调用方持有 SysfsPwm。

#ifndef SERVO_SERVO_H_
#define SERVO_SERVO_H_

#include "math/angle.h"
#include "pwm/sysfs.h"

namespace servo {

class Servo {
 public:
  explicit Servo(SysfsPwm* pwm, int period_us = kDefaultPeriodUs,
                 int min_pulse_us = kDefaultMinPulseUs, int max_pulse_us = kDefaultMaxPulseUs);

  /// 转到 [0, 180] 并保持 PWM。越界或 IO 失败返回 false。
  bool SetAngle(double angle_deg);

  /// 停止 PWM，避免退出后舵机持续受力。
  bool Close();

 private:
  SysfsPwm* pwm_;
  int period_us_;
  int min_pulse_us_;
  int max_pulse_us_;
};

}  // namespace servo

#endif  // SERVO_SERVO_H_
