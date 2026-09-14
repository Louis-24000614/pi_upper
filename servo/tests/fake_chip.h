#ifndef SERVO_TESTS_FAKE_CHIP_H_
#define SERVO_TESTS_FAKE_CHIP_H_

#include <cstdlib>
#include <fstream>
#include <string>
#include <sys/stat.h>
#include <unistd.h>

namespace servo::test {

inline std::string MakeFakeChip() {
  char tmpl[] = "/tmp/servo_chipXXXXXX";
  if (mkdtemp(tmpl) == nullptr) {
    std::abort();
  }
  const std::string chip = std::string(tmpl) + "/pwmchip2";
  if (mkdir(chip.c_str(), 0755) != 0) {
    std::abort();
  }
  std::ofstream(chip + "/npwm") << "1\n";
  std::ofstream(chip + "/export");
  std::ofstream(chip + "/unexport");
  const std::string ch = chip + "/pwm0";
  if (mkdir(ch.c_str(), 0755) != 0) {
    std::abort();
  }
  std::ofstream(ch + "/period") << "0\n";
  std::ofstream(ch + "/duty_cycle") << "0\n";
  std::ofstream(ch + "/enable") << "0\n";
  std::ofstream(ch + "/polarity") << "inversed\n";
  return chip;
}

inline std::string ReadTrim(const std::string& path) {
  std::ifstream in(path);
  std::string line;
  std::getline(in, line);
  while (!line.empty() && (line.back() == '\n' || line.back() == '\r')) {
    line.pop_back();
  }
  return line;
}

inline std::string WriteConfig(const std::string& chip) {
  char tmpl[] = "/tmp/servo_cfgXXXXXX";
  if (mkdtemp(tmpl) == nullptr) {
    std::abort();
  }
  const std::string path = std::string(tmpl) + "/servo.yaml";
  std::ofstream out(path);
  out << "pwmchip: " << chip << "\n"
      << "channel: 0\n"
      << "period_us: 20000\n"
      << "min_pulse_us: 500\n"
      << "max_pulse_us: 2500\n";
  return path;
}

}  // namespace servo::test

#endif  // SERVO_TESTS_FAKE_CHIP_H_
