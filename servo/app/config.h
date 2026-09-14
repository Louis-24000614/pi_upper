/// @file
/// 读取 config/servo.yaml 的扁平 key: value（不引入 YAML 库）。

#ifndef SERVO_APP_CONFIG_H_
#define SERVO_APP_CONFIG_H_

#include "math/angle.h"

#include <string>

namespace servo {

struct ServoConfig {
  std::string pwmchip;
  int channel = 0;
  int period_us = kDefaultPeriodUs;
  int min_pulse_us = kDefaultMinPulseUs;
  int max_pulse_us = kDefaultMaxPulseUs;
};

/// 解析简单 YAML。缺少必填键返回 false。
bool LoadConfig(const std::string& path, ServoConfig* out);

}  // namespace servo

#endif  // SERVO_APP_CONFIG_H_
