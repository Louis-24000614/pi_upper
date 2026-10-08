/// @file
/// 命令行：读配置、设角、保持、关闭 PWM。不属于 libservo。

#ifndef SERVO_APP_CLI_H_
#define SERVO_APP_CLI_H_

#include <iosfwd>

namespace servo {

class Servo;

/// 常驻命令流；EOF/close 均关闭 PWM。供 CLI 与假 sysfs 离线验证使用。
int RunPersistent(Servo* servo, std::istream& input, std::ostream& output);

/// 解析 argv 并驱动舵机。成功返回 0。
int Run(int argc, char** argv);

}  // namespace servo

#endif  // SERVO_APP_CLI_H_
