/// @file
/// 见 app/cli.h。

#include "app/cli.h"

#include <chrono>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>
#include <csignal>
#include <sstream>
#ifndef _WIN32
#include <poll.h>
#include <unistd.h>
#endif

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
            << "       " << argv0 << " --stdin [--config PATH]\n"
            << "  Drive a 180° hobby servo via sysfs PWM (PWM14_M0; verify chip path).\n"
            << "  Default --hold-s is 3 so the horn can finish moving.\n";
}

volatile std::sig_atomic_t bridge_stop = 0;
void BridgeSignal(int) { bridge_stop = 1; }

bool PersistentLine(const std::string& line, Servo* servo, std::ostream& output) {
  std::istringstream in(line);
  std::string command, token, extra;
  in >> command;
  if (command == "close" && !(in >> extra)) {
    return false;
  }
  double angle = 0;
  if (command != "angle" || !(in >> token >> angle) || (in >> extra)) {
    output << "SERVO_FAIL " << (token.empty() ? "unknown" : token) << " invalid_command\n" << std::flush;
    return true;
  }
  if (servo->SetAngle(angle)) {
    output << "SERVO_APPLIED " << token << '\n' << std::flush;
  } else {
    output << "SERVO_FAIL " << token << " pwm_write_failed\n" << std::flush;
  }
  return true;
}

int RunStdinBridge(Servo* servo) {
  bridge_stop = 0;
  const auto before_int = std::signal(SIGINT, BridgeSignal);
  const auto before_term = std::signal(SIGTERM, BridgeSignal);
#ifndef _WIN32
  const auto before_hup = std::signal(SIGHUP, BridgeSignal);
  std::string pending;
  bool running = true;
  while (running && !bridge_stop) {
    struct pollfd fd {STDIN_FILENO, POLLIN, 0};
    const int ready = poll(&fd, 1, 100);
    if (ready < 0) {
      if (!bridge_stop) continue;
      break;
    }
    if (ready == 0) continue;
    char data[512];
    const auto count = read(STDIN_FILENO, data, sizeof(data));
    if (count <= 0) break;
    pending.append(data, static_cast<size_t>(count));
    if (pending.size() > 8192) break;
    size_t newline = 0;
    while (running && (newline = pending.find('\n')) != std::string::npos) {
      running = PersistentLine(pending.substr(0, newline), servo, std::cout);
      pending.erase(0, newline + 1);
    }
  }
  const bool closed = servo->Close();
  std::cout << (closed ? "SERVO_CLOSED\n" : "SERVO_CLOSE_FAILED\n") << std::flush;
  std::signal(SIGHUP, before_hup);
  const int result = closed ? 0 : 1;
#else
  const int result = RunPersistent(servo, std::cin, std::cout);
#endif
  std::signal(SIGINT, before_int);
  std::signal(SIGTERM, before_term);
  return result;
}

}  // namespace

int RunPersistent(Servo* servo, std::istream& input, std::ostream& output) {
  if (servo == nullptr) return 1;
  std::string line;
  while (std::getline(input, line)) {
    if (!PersistentLine(line, servo, output)) break;
  }
  const bool closed = servo->Close();
  output << (closed ? "SERVO_CLOSED\n" : "SERVO_CLOSE_FAILED\n") << std::flush;
  return closed ? 0 : 1;
}

int Run(int argc, char** argv) {
  double angle = 0;
  bool have_angle = false;
  bool stdin_mode = false;
  std::string config_path = SERVO_DEFAULT_CONFIG;
  double hold_s = 3.0;

  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0) {
      Usage(argv[0]);
      return 0;
    }
    if (std::strcmp(argv[i], "--stdin") == 0) {
      stdin_mode = true;
      continue;
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
  if ((!have_angle && !stdin_mode) || (have_angle && stdin_mode)) {
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
  if (stdin_mode) return RunStdinBridge(&servo);
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
