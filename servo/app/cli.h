/// @file
/// 命令行：读配置、设角、保持、关闭 PWM。不属于 libservo。

#ifndef SERVO_APP_CLI_H_
#define SERVO_APP_CLI_H_

namespace servo {

/// 解析 argv 并驱动舵机。成功返回 0。
int Run(int argc, char** argv);

}  // namespace servo

#endif  // SERVO_APP_CLI_H_
