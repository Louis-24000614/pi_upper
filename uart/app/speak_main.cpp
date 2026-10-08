/// @file
/// 单独测试喇叭：发一次 SPEAK_AUDIO，等下位机 ACK。
///
/// 不发 ARM、CMD_VEL、MOTION_ACTION，退出时也不 DISARM，避免碰到运动主流程。
/// ACK_OK 只表示下位机已收下并尝试经 UART4 送给语音模块，不表示播放结束。

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

const char* AckName(uart::AckResult result) {
  switch (result) {
    case uart::AckResult::kOk:
      return "ACK_OK";
    case uart::AckResult::kDeniedState:
      return "DENIED_STATE";
    case uart::AckResult::kDeniedConfig:
      return "DENIED_CONFIG";
    case uart::AckResult::kBadPayload:
      return "BAD_PAYLOAD";
    case uart::AckResult::kUnsupported:
      return "UNSUPPORTED";
    case uart::AckResult::kBusy:
      return "BUSY";
    case uart::AckResult::kVersionMismatch:
      return "VERSION_MISMATCH";
  }
  return "UNKNOWN";
}

void PrintUsage() {
  std::cerr << "用法: uart_speak <1-12> [--device /dev/ttyS6] [--baud 921600]\n";
}

}  // namespace

int main(int argc, char** argv) {
  std::string device = "/dev/ttyS6";
  unsigned baud = 921600;
  uint8_t audio_id = 0;
  bool have_id = false;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--device" && i + 1 < argc) {
      device = argv[++i];
    } else if (arg == "--baud" && i + 1 < argc) {
      baud = static_cast<unsigned>(std::stoul(argv[++i]));
    } else if (!arg.empty() && arg[0] != '-' && !have_id) {
      const unsigned value = static_cast<unsigned>(std::stoul(arg));
      if (value < 1 || value > 12) {
        std::cerr << "音频编号只能是 1 到 12\n";
        return 2;
      }
      audio_id = static_cast<uint8_t>(value);
      have_id = true;
    } else {
      PrintUsage();
      return 2;
    }
  }
  if (!have_id) {
    PrintUsage();
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
  uint32_t timeouts_before = 0;
  const uint64_t t0 = clock.NowMs();
  int exit_code = 0;

  while (!g_stop.load()) {
    session.Poll();
    const uint64_t now = clock.NowMs();

    if (!sent && session.link_state() == uart::LinkState::kConnected && !session.request_pending()) {
      timeouts_before = session.diagnostics().ack_timeouts;
      if (!session.RequestSpeech(audio_id)) {
        std::cerr << "语音指令被拒绝\n";
        exit_code = 1;
        break;
      }
      sent = true;
      std::cerr << "SPEAK " << static_cast<int>(audio_id) << "\n";
    }

    if (sent && !session.request_pending()) {
      const uart::Telemetry& telemetry = session.telemetry();
      if (telemetry.has_ack &&
          telemetry.last_ack.request_type == static_cast<uint8_t>(uart::MsgType::kSpeakAudio)) {
        const uart::AckResult result = telemetry.last_ack.result;
        if (result == uart::AckResult::kOk) {
          std::cout << "ACK_OK\n";
        } else {
          std::cerr << AckName(result) << "\n";
          exit_code = 1;
        }
        break;
      }
      if (session.diagnostics().ack_timeouts > timeouts_before) {
        std::cerr << "等待 ACK 超时\n";
        exit_code = 1;
        break;
      }
    }

    if (!sent && now - t0 > 5000) {
      std::cerr << "等待建链超时\n";
      exit_code = 1;
      break;
    }
    if (sent && now - t0 > 5000) {
      std::cerr << "等待 ACK 超时\n";
      exit_code = 1;
      break;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  if (g_stop.load()) {
    return 1;
  }
  return exit_code;
}
