/// @file
/// LoadConfig 与 CLI Run。链 servo_app，不混进 math 测试。

#include "app/cli.h"
#include "app/config.h"

#include <string>
#include <vector>

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

}  // namespace

int main() {
  TestLoadConfig();
  TestCliSetsThenDisables();
  return servo::test::Finish("app");
}
