/// @file
/// Servo::SetAngle 经注入 PWM 写出脉宽。

#include "servo.h"

#include "check.h"
#include "fake_chip.h"

namespace {

void TestSetAngle90() {
  const std::string chip = servo::test::MakeFakeChip();
  servo::SysfsPwm pwm(chip, 0);
  servo::Servo servo(&pwm, 20000, 500, 2500);
  CHECK(servo.SetAngle(90));
  CHECK(servo::test::ReadTrim(chip + "/pwm0/duty_cycle") == "1500000");
  CHECK(servo::test::ReadTrim(chip + "/pwm0/period") == "20000000");
}

void TestCloseDisables() {
  const std::string chip = servo::test::MakeFakeChip();
  servo::SysfsPwm pwm(chip, 0);
  servo::Servo servo(&pwm);
  CHECK(servo.SetAngle(0));
  CHECK(servo.Close());
  CHECK(servo::test::ReadTrim(chip + "/pwm0/enable") == "0");
}

}  // namespace

int main() {
  TestSetAngle90();
  TestCloseDisables();
  return servo::test::Finish("servo");
}
