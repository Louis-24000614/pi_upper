/// @file
/// 见 pwm/sysfs.h。

#include "pwm/sysfs.h"

#include <fstream>
#include <sys/stat.h>

namespace servo {
namespace {

bool IsDir(const std::string& path) {
  struct stat st {};
  return stat(path.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
}

}  // namespace

SysfsPwm::SysfsPwm(std::string chip, int channel)
    : chip_(std::move(chip)), channel_(channel), dir_(chip_ + "/pwm" + std::to_string(channel_)) {}

bool SysfsPwm::Configure(int64_t period_ns, int64_t duty_ns) {
  if (!EnsureExported()) {
    return false;
  }
  // 新导出的 RK3588 通道周期可能为 0；此时写 enable/polarity 会返回 EINVAL。
  // 先建立有效周期，再沿用关闭、极性、周期和脉宽的配置流程。
  int64_t current_period = 0;
  std::ifstream period(dir_ + "/period");
  if (!(period >> current_period)) {
    return false;
  }
  if (current_period == 0 && !Write("period", std::to_string(period_ns) + "\n")) {
    return false;
  }
  if (!Write("enable", "0\n")) {
    return false;
  }
  if (!Write("polarity", "normal\n")) {
    return false;
  }
  if (!Write("period", std::to_string(period_ns) + "\n")) {
    return false;
  }
  if (!Write("duty_cycle", std::to_string(duty_ns) + "\n")) {
    return false;
  }
  return Write("enable", "1\n");
}

bool SysfsPwm::Disable() {
  if (!IsDir(dir_)) {
    return true;
  }
  return Write("enable", "0\n");
}

bool SysfsPwm::EnsureExported() {
  if (IsDir(dir_)) {
    return true;
  }
  std::ofstream out(chip_ + "/export");
  if (!out) {
    return false;
  }
  out << channel_ << '\n';
  out.flush();
  return IsDir(dir_);
}

bool SysfsPwm::Write(const char* name, const std::string& value) const {
  std::ofstream out(dir_ + "/" + name);
  if (!out) {
    return false;
  }
  out << value;
  out.flush();
  return static_cast<bool>(out);
}

}  // namespace servo
