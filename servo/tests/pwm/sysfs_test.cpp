/// @file
/// 假 sysfs：写出 period / duty_cycle / polarity / enable。

#include "pwm/sysfs.h"

#include "check.h"
#include "fake_chip.h"

namespace {

void TestConfigureWritesNormalPolarity() {
  const std::string chip = servo::test::MakeFakeChip();
  servo::SysfsPwm pwm(chip, 0);
  CHECK(pwm.Configure(20000000, 1500000));
  CHECK(servo::test::ReadTrim(chip + "/pwm0/period") == "20000000");
  CHECK(servo::test::ReadTrim(chip + "/pwm0/duty_cycle") == "1500000");
  CHECK(servo::test::ReadTrim(chip + "/pwm0/polarity") == "normal");
  CHECK(servo::test::ReadTrim(chip + "/pwm0/enable") == "1");
}

void TestDisable() {
  const std::string chip = servo::test::MakeFakeChip();
  servo::SysfsPwm pwm(chip, 0);
  CHECK(pwm.Configure(20000000, 500000));
  CHECK(pwm.Disable());
  CHECK(servo::test::ReadTrim(chip + "/pwm0/enable") == "0");
}

}  // namespace

int main() {
  TestConfigureWritesNormalPolarity();
  TestDisable();
  return servo::test::Finish("pwm");
}
