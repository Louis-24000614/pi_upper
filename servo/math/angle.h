/// @file
/// 将 0–180° 舵机角度线性映射为 PWM 脉宽（纳秒）。无 IO。

#ifndef SERVO_MATH_ANGLE_H_
#define SERVO_MATH_ANGLE_H_

#include <cstdint>

namespace servo {

constexpr int kUsToNs = 1000;
constexpr double kAngleMinDeg = 0.0;
constexpr double kAngleMaxDeg = 180.0;
constexpr int kDefaultMinPulseUs = 500;
constexpr int kDefaultMaxPulseUs = 2500;
constexpr int kDefaultPeriodUs = 20000;

/// 周期（微秒）转为 sysfs 使用的纳秒。
int64_t PeriodNs(int period_us = kDefaultPeriodUs);

/// 线性映射角度到脉宽（纳秒）。
///
/// @p angle_deg 必须在闭区间 [0, 180]。成功返回 true 并写入 @p out_pulse_ns。
bool AngleToPulseNs(double angle_deg, int64_t* out_pulse_ns,
                    int min_pulse_us = kDefaultMinPulseUs,
                    int max_pulse_us = kDefaultMaxPulseUs);

}  // namespace servo

#endif  // SERVO_MATH_ANGLE_H_
