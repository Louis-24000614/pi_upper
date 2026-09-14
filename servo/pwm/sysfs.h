/// @file
/// Linux sysfs PWM 通道。chip 路径可注入；不在用户态伪造 pwmX 目录。

#ifndef SERVO_PWM_SYSFS_H_
#define SERVO_PWM_SYSFS_H_

#include <cstdint>
#include <string>

namespace servo {

class SysfsPwm {
 public:
  SysfsPwm(std::string chip, int channel);

  /// 导出通道（如需），设 polarity=normal，写入周期与脉宽并使能。
  bool Configure(int64_t period_ns, int64_t duty_ns);

  /// 停止 PWM（enable=0）。通道未导出时视为成功。
  bool Disable();

 private:
  bool EnsureExported();
  bool Write(const char* name, const std::string& value) const;

  std::string chip_;
  int channel_;
  std::string dir_;
};

}  // namespace servo

#endif  // SERVO_PWM_SYSFS_H_
