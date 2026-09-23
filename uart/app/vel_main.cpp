/// @file
/// 速度环桥：从标准输入读 ``v w``，经会话层发 CMD_VEL。
///
/// 寻线进程把每一帧的速度写进来。本进程负责 HELLO / ARM、50 Hz 节拍，
/// 以及退出时发零速并 DISARM。标准输入关闭或收到 SIGINT/SIGTERM 时停车。

#include <fcntl.h>
#include <signal.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

#include "link/clock.h"
#include "link/port.h"
#include "link/sess.h"

namespace {

std::atomic<bool> g_stop{false};

void OnSignal(int) { g_stop.store(true); }

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

  while (!g_stop.load()) {
    session.Poll();
    if (session.link_state() == uart::LinkState::kConnected && !session.command_enabled() &&
        !session.request_pending()) {
      session.RequestArm();
    }
    if (!announced && session.command_enabled()) {
      std::cerr << "ARMED " << device << "\n";
      announced = true;
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
        std::cerr << "LINK 已连上 " << device << "，等待使能\n";
      } else {
        std::cerr << "LINK 等待下位机 " << device << "\n";
      }
    }
    if (have_cmd && now_ms - cmd_ms <= 250) {
      session.SetVelocity(linear, angular);
    } else if (have_cmd) {
      session.SetVelocity(0.0f, 0.0f);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  session.Shutdown();
  std::cerr << "STOP\n";
  return 0;
}
