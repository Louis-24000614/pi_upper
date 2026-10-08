/// @file
/// LoadConfig 与 CLI Run。链 servo_app，不混进 math 测试。

#include "app/cli.h"
#include "app/config.h"

#include <string>
#include <vector>
#include <sstream>
#include "servo.h"
#include "pwm/sysfs.h"

#include "check.h"
#include "fake_chip.h"

namespace {

std::vector<char*> MakeArgv(std::vector<std::string>& args) {
  std::vector<char*> argv;
  argv.reserve(args.size());
  for (auto& s : args) {
    argv.push_back(s.data());
  }
  return argv;
}

void TestLoadConfig() {
  const std::string chip = servo::test::MakeFakeChip();
  const std::string path = servo::test::WriteConfig(chip);
  servo::ServoConfig cfg;
  CHECK(servo::LoadConfig(path, &cfg));
  CHECK(cfg.pwmchip == chip);
  CHECK(cfg.channel == 0);
  CHECK(cfg.period_us == 20000);
  CHECK(cfg.min_pulse_us == 500);
  CHECK(cfg.max_pulse_us == 2500);
}

void TestCliSetsThenDisables() {
  const std::string chip = servo::test::MakeFakeChip();
  const std::string path = servo::test::WriteConfig(chip);
  std::vector<std::string> args = {"servo_cli", "--angle", "90", "--config", path, "--hold-s", "0"};
  auto argv = MakeArgv(args);
  CHECK(servo::Run(static_cast<int>(argv.size()), argv.data()) == 0);
  CHECK(servo::test::ReadTrim(chip + "/pwm0/duty_cycle") == "1500000");
  CHECK(servo::test::ReadTrim(chip + "/pwm0/enable") == "0");
}

void TestPersistentTwoEndpointsAndClose() {
  const std::string chip = servo::test::MakeFakeChip();
  servo::SysfsPwm pwm(chip, 0);
  servo::Servo servo(&pwm);
  std::istringstream input("angle first 0\nangle second 180\nclose\n");
  std::ostringstream output;
  CHECK(servo::RunPersistent(&servo, input, output) == 0);
  CHECK(output.str() == "SERVO_APPLIED first\nSERVO_APPLIED second\nSERVO_CLOSED\n");
  CHECK(servo::test::ReadTrim(chip + "/pwm0/duty_cycle") == "2500000");
  CHECK(servo::test::ReadTrim(chip + "/pwm0/enable") == "0");
}

void TestPersistentInvalidAngleAndEofClose() {
  const std::string chip = servo::test::MakeFakeChip();
  servo::SysfsPwm pwm(chip, 0);
  servo::Servo servo(&pwm);
  std::istringstream input("angle good 90\nangle bad 181\n");
  std::ostringstream output;
  CHECK(servo::RunPersistent(&servo, input, output) == 0);
  CHECK(output.str().find("SERVO_FAIL bad pwm_write_failed") != std::string::npos);
  CHECK(servo::test::ReadTrim(chip + "/pwm0/enable") == "0");
}

}  // namespace

int main() {
  TestLoadConfig();
  TestCliSetsThenDisables();
  TestPersistentTwoEndpointsAndClose();
  TestPersistentInvalidAngleAndEofClose();
  return servo::test::Finish("app");
}
