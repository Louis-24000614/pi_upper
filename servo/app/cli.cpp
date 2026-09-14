/// @file
/// 见 app/cli.h。

#include "app/cli.h"

#include <chrono>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

#include "app/config.h"
#include "pwm/sysfs.h"
#include "servo.h"

#ifndef SERVO_DEFAULT_CONFIG
#define SERVO_DEFAULT_CONFIG "config/servo.yaml"
#endif

namespace servo {
namespace {

void Usage(const char* argv0) {
  std::cerr << "Usage: " << argv0
            << " --angle DEG [--config PATH] [--hold-s SEC]\n"
            << "  Drive a 180° hobby servo via sysfs PWM (Pin 7 / PWM14_M2).\n"
            << "  Default --hold-s is 3 so the horn can finish moving.\n";
}

}  // namespace

int Run(int argc, char** argv) {
  double angle = 0;
  bool have_angle = false;
  std::string config_path = SERVO_DEFAULT_CONFIG;
  double hold_s = 3.0;

  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0) {
      Usage(argv[0]);
      return 0;
    }
    if (std::strcmp(argv[i], "--angle") == 0 && i + 1 < argc) {
      angle = std::stod(argv[++i]);
      have_angle = true;
      continue;
    }
    if (std::strcmp(argv[i], "--config") == 0 && i + 1 < argc) {
      config_path = argv[++i];
      continue;
    }
    if (std::strcmp(argv[i], "--hold-s") == 0 && i + 1 < argc) {
      hold_s = std::stod(argv[++i]);
      continue;
    }
    Usage(argv[0]);
    return 2;
  }
  if (!have_angle) {
    Usage(argv[0]);
    return 2;
  }

  ServoConfig cfg;
  if (!LoadConfig(config_path, &cfg)) {
    std::cerr << "failed to load config: " << config_path << '\n';
    return 1;
  }
  SysfsPwm pwm(cfg.pwmchip, cfg.channel);
  Servo servo(&pwm, cfg.period_us, cfg.min_pulse_us, cfg.max_pulse_us);
  const bool ok = servo.SetAngle(angle);
  if (!ok) {
    std::cerr << "failed to set angle " << angle << '\n';
  } else {
    std::cout << "angle=" << angle << " hold-s=" << hold_s << '\n';
  }
  if (hold_s > 0) {
    std::this_thread::sleep_for(std::chrono::duration<double>(hold_s));
  }
  servo.Close();
  return ok ? 0 : 1;
}

}  // namespace servo
