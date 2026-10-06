/// @file
/// 速度环桥：从标准输入读 ``v w``，经会话层发 CMD_VEL。
///
/// 寻线进程把每一帧的速度写进来。本进程负责 HELLO / ARM、50 Hz 节拍，
/// 以及退出时发零速并 DISARM。标准输入关闭或收到 SIGINT/SIGTERM 时停车。

#include <fcntl.h>
#include <signal.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
#include <thread>

#include "link/clock.h"
#include "link/port.h"
#include "link/sess.h"

namespace {

std::atomic<bool> g_stop{false};

void OnSignal(int) { g_stop.store(true); }

enum class FiniteAction { kNone, kForward, kBackward, kTurn, kStopping };

}  // namespace

int main(int argc, char** argv) {
  std::string device = "/dev/ttyS6";
  unsigned baud = 921600;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--device" && i + 1 < argc) {
      device = argv[++i];
    } else if (arg == "--baud" && i + 1 < argc) {
      baud = static_cast<unsigned>(std::stoul(argv[++i]));
    } else {
      std::cerr << "用法: uart_vel --device /dev/ttyS6 [--baud 921600]\n";
      return 2;
    }
  }

  uart::SerialPort port;
  uart::SerialConfig serial;
  serial.device = device;
  serial.baud = baud;
  if (!port.Open(serial)) {
    std::cerr << port.last_error() << "\n";
    return 1;
  }

  ::signal(SIGINT, OnSignal);
  ::signal(SIGTERM, OnSignal);

  const int flags = ::fcntl(STDIN_FILENO, F_GETFL, 0);
  if (flags >= 0) {
    ::fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
  }

  uart::SteadyClock clock;
  uart::SessionConfig session_config;
  session_config.cmd_period_ms = 20;
  session_config.cmd_validity_ms = 200;
  uart::Session session(port, clock, session_config);
  session.Start();

  std::string pending;
  bool have_cmd = false;
  float linear = 0.0f;
  float angular = 0.0f;
  uint64_t cmd_ms = 0;
  bool announced = false;
  uint64_t last_status_ms = 0;
  uint64_t last_odom_ms = 0;
  uint64_t seen_odom_us = 0;
  FiniteAction finite_action = FiniteAction::kNone;
  uint64_t action_ms = 0;
  uint64_t action_timeout_ms = 20000;
  uint64_t seen_motion_us = 0;
  uint64_t seen_rfid_us = 0;
  bool have_rfid_state = false;
  uint8_t last_rfid_present = 0;
  uint8_t last_rfid_number = 0;
  uint8_t last_rfid_generation = 0;

  while (!g_stop.load()) {
    session.Poll();
    if (session.link_state() == uart::LinkState::kConnected && !session.command_enabled() &&
        !session.request_pending()) {
      session.RequestArm();
    }
    if (!announced && session.command_enabled()) {
      std::cerr << "[串口] 已使能 " << device << "\n";
      announced = true;
    }

    const uart::Telemetry& telemetry = session.telemetry();
    if (telemetry.has_rfid && telemetry.rfid_us != seen_rfid_us) {
      seen_rfid_us = telemetry.rfid_us;
      const uart::RfidCard& rfid = telemetry.rfid;
      const bool valid = rfid.present != 0 && rfid.card_number >= 1 && rfid.card_number <= 12;
      if (valid && (!have_rfid_state || rfid.generation != last_rfid_generation)) {
        std::cout << "RFID_EVENT " << static_cast<unsigned>(rfid.card_number) << " "
                  << static_cast<unsigned>(rfid.generation) << "\n" << std::flush;
      } else if (have_rfid_state && last_rfid_present != 0 && rfid.present == 0) {
        std::cout << "RFID_REMOVED " << static_cast<unsigned>(last_rfid_generation) << "\n"
                  << std::flush;
      } else if (rfid.present != 0 && rfid.card_number == 0 &&
                 (!have_rfid_state || last_rfid_present == 0 || last_rfid_number != 0)) {
        std::cout << "RFID_INVALID " << static_cast<unsigned>(rfid.generation) << "\n"
                  << std::flush;
      }
      have_rfid_state = true;
      last_rfid_present = rfid.present;
      last_rfid_number = rfid.card_number;
      last_rfid_generation = rfid.generation;
    }

    char buf[256];
    const ssize_t n = ::read(STDIN_FILENO, buf, sizeof(buf));
    if (n < 0) {
      if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
        std::cerr << "读取速度指令失败\n";
        break;
      }
    } else if (n == 0) {
      break;
    } else {
      pending.append(buf, buf + n);
      std::size_t pos = 0;
      while ((pos = pending.find('\n')) != std::string::npos) {
        const std::string line = pending.substr(0, pos);
        pending.erase(0, pos + 1);
        if (line == "stop") {
          const bool was_finite = finite_action != FiniteAction::kNone;
          have_cmd = false;
          seen_motion_us = session.telemetry().motion_us;
          if (!session.RequestMotionAction(static_cast<uint8_t>(uart::MotionActionId::kStop))) {
            std::cout << "STOP_FAIL\n" << std::flush;
          } else if (was_finite) {
            finite_action = FiniteAction::kStopping;
            action_ms = clock.NowMs();
            action_timeout_ms = 2000;
          } else {
            std::cout << "STOP_DONE\n" << std::flush;
          }
          continue;
        }
        if (line == "turn left" || line == "turn right") {
          if (finite_action == FiniteAction::kNone) {
            const auto action = line == "turn left" ? uart::MotionActionId::kTurnLeft
                                                     : uart::MotionActionId::kTurnRight;
            have_cmd = false;
            seen_motion_us = session.telemetry().motion_us;
            if (!session.RequestMotionAction(static_cast<uint8_t>(action), 1)) {
              std::cout << "TURN_FAIL\n" << std::flush;
            } else {
              finite_action = FiniteAction::kTurn;
              action_ms = clock.NowMs();
              action_timeout_ms = 20000;
              std::cerr << (line == "turn left" ? "[动作] 开始原地左转\n"
                                                  : "[动作] 开始原地右转\n");
            }
          }
          continue;
        }
        if (line.rfind("forward ", 0) == 0 || line.rfind("backward ", 0) == 0) {
          const bool backward = line.rfind("backward ", 0) == 0;
          std::istringstream in(line);
          std::string verb;
          uint32_t distance_mm = 0;
          uint32_t speed_mmps = 0;
          std::string extra;
          const bool valid = (in >> verb >> distance_mm >> speed_mmps) && !(in >> extra) &&
                             distance_mm >= 1 && distance_mm <= 1000 &&
                             speed_mmps >= 20 && speed_mmps <= 400;
          const char* name = backward ? "BACKWARD" : "FORWARD";
          if (!valid || finite_action != FiniteAction::kNone) {
            std::cout << name << "_FAIL\n" << std::flush;
          } else {
            have_cmd = false;
            seen_motion_us = session.telemetry().motion_us;
            const auto action = backward ? uart::MotionActionId::kBackward
                                         : uart::MotionActionId::kForward;
            const bool sent = session.RequestMotionAction(
                static_cast<uint8_t>(action), 0, static_cast<uint16_t>(speed_mmps), distance_mm);
            if (!sent) {
              std::cout << name << "_FAIL\n" << std::flush;
            } else {
              finite_action = backward ? FiniteAction::kBackward : FiniteAction::kForward;
              action_ms = clock.NowMs();
              const uint64_t expected_ms =
                  static_cast<uint64_t>(distance_mm) * 1000ULL / speed_mmps;
              action_timeout_ms = std::min<uint64_t>(
                  30000, std::max<uint64_t>(5000, expected_ms * 3 + 2000));
              std::cerr << "[动作] 开始定距" << (backward ? "后退 " : "前进 ")
                        << distance_mm << " mm，速度 " << speed_mmps << " mm/s\n";
            }
          }
          continue;
        }
        float next_v = 0.0f;
        float next_w = 0.0f;
        std::istringstream in(line);
        if (!(in >> next_v >> next_w)) {
          next_v = 0.0f;
          next_w = 0.0f;
        }
        linear = next_v;
        angular = next_w;
        have_cmd = true;
        cmd_ms = clock.NowMs();
      }
    }

    const uint64_t now_ms = clock.NowMs();
    if (!session.command_enabled() && now_ms - last_status_ms >= 1000) {
      last_status_ms = now_ms;
      if (session.link_state() == uart::LinkState::kConnected) {
        std::cerr << "[串口] 已连接 " << device << "，等待使能\n";
      } else {
        std::cerr << "[串口] 等待下位机 " << device << "\n";
      }
    }
    if (finite_action != FiniteAction::kNone) {
      const uart::Telemetry& tel = session.telemetry();
      const bool denied = !session.request_pending() && tel.has_ack &&
                          tel.last_ack.request_type ==
                              static_cast<uint8_t>(uart::MsgType::kMotionAction) &&
                          tel.last_ack.result != uart::AckResult::kOk;
      const bool finished = tel.has_motion && tel.motion_us != seen_motion_us &&
                            !session.awaiting_motion_result();
      const char* prefix = finite_action == FiniteAction::kForward
                               ? "FORWARD"
                               : (finite_action == FiniteAction::kBackward
                                      ? "BACKWARD"
                                      : (finite_action == FiniteAction::kTurn ? "TURN" : "STOP"));
      if (denied || now_ms - action_ms > action_timeout_ms) {
        if (!denied) {
          session.RequestMotionAction(static_cast<uint8_t>(uart::MotionActionId::kStop));
        }
        finite_action = FiniteAction::kNone;
        std::cout << prefix << "_FAIL\n" << std::flush;
      } else if (finished) {
        const bool stopping_action = finite_action == FiniteAction::kStopping;
        finite_action = FiniteAction::kNone;
        const bool ok = stopping_action || tel.motion.result == uart::MotionResultCode::kCompleted;
        std::cout << prefix << (ok ? "_DONE\n" : "_FAIL\n") << std::flush;
      }
    } else if (have_cmd && now_ms - cmd_ms <= 250) {
      session.SetVelocity(linear, angular);
    } else if (have_cmd) {
      session.SetVelocity(0.0f, 0.0f);
    }

    const uart::Telemetry& odom_tel = session.telemetry();
    if (odom_tel.has_odom && (odom_tel.odom.status_flags & uart::kOdomValid) != 0 &&
        odom_tel.odom_us != seen_odom_us &&
        now_ms - last_odom_ms >= 50) {
      last_odom_ms = now_ms;
      seen_odom_us = odom_tel.odom_us;
      const uart::OdomState& odom = odom_tel.odom;
      std::ostringstream line;
      line << std::fixed << std::setprecision(3) << "ODOM " << odom.x_m << " " << odom.y_m << " "
           << odom.yaw_rad << " 1\n";
      std::cout << line.str() << std::flush;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  session.Shutdown();
  std::cerr << "[串口] 已停车\n";
  return 0;
}
