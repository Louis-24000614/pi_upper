/// @file
/// 路口左右转：发一次 MOTION_ACTION，等 IMU 转完 90° 的整数倍。
///
/// 不发 CMD_VEL，也不启动寻线。左转、右转与急停都走动作环。

#include <signal.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>
#include <thread>

#include "link/clock.h"
#include "link/port.h"
#include "link/sess.h"

namespace {

std::atomic<bool> g_stop{false};

void OnSignal(int) { g_stop.store(true); }

const char* ResultName(uart::MotionResultCode code) {
  switch (code) {
    case uart::MotionResultCode::kCompleted:
      return "DONE";
    case uart::MotionResultCode::kInterrupted:
      return "INTERRUPTED";
    case uart::MotionResultCode::kImuUnavailable:
      return "IMU_UNAVAILABLE";
  }
  return "UNKNOWN";
}

}  // namespace

int main(int argc, char** argv) {
  std::string device = "/dev/ttyS6";
  unsigned baud = 921600;
  uint8_t action = 0;
  uint8_t quarters = 1;
  bool have_action = false;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--device" && i + 1 < argc) {
      device = argv[++i];
    } else if (arg == "--baud" && i + 1 < argc) {
      baud = static_cast<unsigned>(std::stoul(argv[++i]));
    } else if (arg == "--quarters" && i + 1 < argc) {
      const unsigned value = static_cast<unsigned>(std::stoul(argv[++i]));
      if (value < 1 || value > 4) {
        std::cerr << "quarter 只能是 1 到 4\n";
        return 2;
      }
      quarters = static_cast<uint8_t>(value);
    } else if (arg == "left") {
      action = static_cast<uint8_t>(uart::MotionActionId::kTurnLeft);
      have_action = true;
    } else if (arg == "right") {
      action = static_cast<uint8_t>(uart::MotionActionId::kTurnRight);
      have_action = true;
    } else {
      std::cerr << "用法: uart_turn left|right [--quarters 1] [--device /dev/ttyS6] [--baud 921600]\n";
      return 2;
    }
  }
  if (!have_action) {
    std::cerr << "用法: uart_turn left|right [--quarters 1] [--device /dev/ttyS6] [--baud 921600]\n";
    return 2;
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

  uart::SteadyClock clock;
  uart::Session session(port, clock);
  session.Start();

  bool sent = false;
  const uint64_t t0 = clock.NowMs();
  int exit_code = 0;

  while (!g_stop.load()) {
    session.Poll();
    const uint64_t now = clock.NowMs();

    if (!sent && session.command_enabled() && !session.request_pending()) {
      if (!session.RequestMotionAction(action, quarters)) {
        std::cerr << "转弯指令被拒绝\n";
        exit_code = 1;
        break;
      }
      sent = true;
      std::cerr << (action == static_cast<uint8_t>(uart::MotionActionId::kTurnLeft) ? "TURN_LEFT" : "TURN_RIGHT")
                << " quarters=" << static_cast<int>(quarters) << "\n";
    }

    if (sent && session.telemetry().has_ack &&
        session.telemetry().last_ack.request_type == static_cast<uint8_t>(uart::MsgType::kMotionAction) &&
        session.telemetry().last_ack.result != uart::AckResult::kOk) {
      std::cerr << "ACK " << static_cast<int>(session.telemetry().last_ack.result) << "\n";
      exit_code = 1;
      break;
    }

    if (sent && session.telemetry().has_motion && !session.awaiting_motion_result()) {
      const uart::MotionResult& result = session.telemetry().motion;
      std::cout << ResultName(result.result)
                << " yaw=" << result.final_yaw_rad
                << " target=" << result.target_yaw_rad << "\n";
      if (result.result != uart::MotionResultCode::kCompleted) {
        exit_code = 1;
      }
      break;
    }

    if (!sent && now - t0 > 5000) {
      std::cerr << "等待使能超时\n";
      exit_code = 1;
      break;
    }
    if (sent && now - t0 > 20000) {
      std::cerr << "等待转弯完成超时\n";
      exit_code = 1;
      break;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  session.Shutdown();
  return exit_code;
}
