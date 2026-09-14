/// @file
/// 0–180° 到 PWM 脉宽，无 IO。只链 servo_math。

#include "math/angle.h"

#include "check.h"

namespace {

void TestPeriod20ms() { CHECK(servo::PeriodNs(20000) == 20000 * servo::kUsToNs); }

void TestHobbyEndpoints() {
  int64_t pulse = 0;
  CHECK(servo::AngleToPulseNs(0, &pulse));
  CHECK(pulse == 500 * servo::kUsToNs);
  CHECK(servo::AngleToPulseNs(90, &pulse));
  CHECK(pulse == 1500 * servo::kUsToNs);
  CHECK(servo::AngleToPulseNs(180, &pulse));
  CHECK(pulse == 2500 * servo::kUsToNs);
}

void TestRejectsOutOfRange() {
  int64_t pulse = 0;
  CHECK(!servo::AngleToPulseNs(-0.1, &pulse));
  CHECK(!servo::AngleToPulseNs(180.1, &pulse));
}

}  // namespace

int main() {
  TestPeriod20ms();
  TestHobbyEndpoints();
  TestRejectsOutOfRange();
  return servo::test::Finish("angle");
}
