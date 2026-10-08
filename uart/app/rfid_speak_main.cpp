/// @file
/// 单独测试读卡和对应播报。
///
/// 只发 HELLO 和 SPEAK_AUDIO。不发 ARM、CMD_VEL、MOTION_ACTION，退出时也不 DISARM。
/// 读到有效卡号 1～12 且世代号变化时，播放同编号预录音。同一张卡一直贴着不重复播。

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
  std::cerr << "用法: uart_rfid_speak [--device /dev/ttyS6] [--baud 921600]\n"
            << "贴上 1～12 号卡后播放同编号语音。Ctrl+C 结束。不使能电机。\n";
}

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
    } else if (arg == "--help" || arg == "-h") {
      PrintUsage();
      return 0;
    } else {
      PrintUsage();
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

  uart::SteadyClock clock;
  uart::Session session(port, clock);
  session.Start();

  bool announced = false;
  uint64_t seen_rfid_us = 0;
  bool have_rfid_state = false;
  uint8_t last_present = 0;
  uint8_t last_number = 0;
  uint8_t last_generation = 0;
  bool speak_pending = false;
  uint32_t timeouts_before = 0;
  uint8_t speaking_id = 0;
  uint64_t last_status_ms = clock.NowMs();

  std::cerr << "等待读卡。贴上卡片后会播报对应编号。Ctrl+C 结束。\n";

  while (!g_stop.load()) {
    session.Poll();

    if (!announced && session.link_state() == uart::LinkState::kConnected) {
      announced = true;
      std::cerr << "已连接 " << device << "\n";
    }

    const uart::Telemetry& telemetry = session.telemetry();
    if (telemetry.has_rfid && telemetry.rfid_us != seen_rfid_us) {
      seen_rfid_us = telemetry.rfid_us;
      const uart::RfidCard& rfid = telemetry.rfid;
      const bool valid = rfid.present != 0 && rfid.card_number >= 1 && rfid.card_number <= 12;
      if (valid && (!have_rfid_state || rfid.generation != last_generation)) {
        std::cout << "读到 " << static_cast<unsigned>(rfid.card_number) << " 号标签（第 "
                  << static_cast<unsigned>(rfid.generation) << " 次）\n"
                  << std::flush;
        if (speak_pending || session.request_pending()) {
          std::cerr << "上一条播报还没确认，跳过 " << static_cast<unsigned>(rfid.card_number)
                    << " 号\n";
        } else if (!session.RequestSpeech(rfid.card_number)) {
          std::cerr << "播报 " << static_cast<unsigned>(rfid.card_number) << " 号被拒绝\n";
        } else {
          speak_pending = true;
          speaking_id = rfid.card_number;
          timeouts_before = session.diagnostics().ack_timeouts;
          std::cerr << "播报 " << static_cast<unsigned>(speaking_id) << " 号\n";
        }
      } else if (have_rfid_state && last_present != 0 && rfid.present == 0) {
        std::cout << "标签已移开\n" << std::flush;
      } else if (rfid.present != 0 && rfid.card_number == 0 &&
                 (!have_rfid_state || last_present == 0 || last_number != 0)) {
        std::cout << "读到无效标签\n" << std::flush;
      }
      have_rfid_state = true;
      last_present = rfid.present;
      last_number = rfid.card_number;
      last_generation = rfid.generation;
    }

    const uint64_t now_ms = clock.NowMs();
    if (now_ms - last_status_ms >= 1000) {
      last_status_ms = now_ms;
      if (!have_rfid_state) {
        std::cerr << "还没收到读卡帧\n";
      } else if (last_present == 0) {
        std::cerr << "无卡\n";
      } else if (last_number >= 1 && last_number <= 12) {
        std::cerr << "卡还在 " << static_cast<unsigned>(last_number) << " 号\n";
      } else {
        std::cerr << "卡在场，但卡号无效\n";
      }
    }

    if (speak_pending && !session.request_pending()) {
      speak_pending = false;
      if (telemetry.has_ack &&
          telemetry.last_ack.request_type == static_cast<uint8_t>(uart::MsgType::kSpeakAudio)) {
        const uart::AckResult result = telemetry.last_ack.result;
        if (result == uart::AckResult::kOk) {
          std::cout << "播报 " << static_cast<unsigned>(speaking_id) << " 号已收下\n" << std::flush;
        } else {
          std::cerr << "播报 " << static_cast<unsigned>(speaking_id) << " 号失败: " << AckName(result)
                    << "\n";
        }
      } else if (session.diagnostics().ack_timeouts > timeouts_before) {
        std::cerr << "播报 " << static_cast<unsigned>(speaking_id) << " 号等待确认超时\n";
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  std::cerr << "结束\n";
  return 0;
}
