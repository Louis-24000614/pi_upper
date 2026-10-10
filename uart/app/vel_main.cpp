/// @file
/// 速度环桥：从标准输入读 ``v w``，经会话层发 CMD_VEL。
///
/// 寻线进程把每一帧的速度写进来。本进程负责 HELLO / ARM、50 Hz 节拍，
/// 以及退出时发零速并 DISARM。读到 1～12 号新标签时播报同编号语音。
/// 标准输入关闭或收到 SIGINT/SIGTERM 时停车。

#include <fcntl.h>
#include <signal.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <limits>
#include <iomanip>
#include <sstream>
#include <string>
#include <thread>

#include "link/clock.h"
#include "link/port.h"
#include "link/sess.h"
#include "link/speech_queue.h"

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

enum class FiniteAction { kNone, kForward, kBackward, kTurn, kStopping };

const char* CompletionName(const uart::RequestCompletion& completion) {
  switch (completion.code) {
    case uart::RequestCompletionCode::kAck: return AckName(completion.result);
    case uart::RequestCompletionCode::kTimeout: return "ACK_TIMEOUT";
    case uart::RequestCompletionCode::kLinkLost: return "LINK_LOST";
    case uart::RequestCompletionCode::kCancelled: return "CANCELLED";
  }
  return "UNKNOWN";
}

}  // namespace

int main(int argc, char** argv) {
  std::string device = "/dev/ttyS6";
  unsigned baud = 921600;
  bool require_heading_anchor = false;
  uart::SpeechQueue speech_queue;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--device" && i + 1 < argc) {
      device = argv[++i];
    } else if (arg == "--baud" && i + 1 < argc) {
      baud = static_cast<unsigned>(std::stoul(argv[++i]));
    } else if (arg == "--require-heading-anchor") {
      require_heading_anchor = true;
    } else if (arg == "--speech-hold-ms" && i + 1 < argc) {
      std::istringstream spec(argv[++i]);
      unsigned id = 0;
      uint64_t ms = 0;
      char colon = 0;
      std::string extra;
      if (!(spec >> id >> colon >> ms) || colon != ':' || (spec >> extra) ||
          ms > std::numeric_limits<uint32_t>::max() ||
          !speech_queue.SetDuration(id, static_cast<uint32_t>(ms))) {
        std::cerr << "Invalid --speech-hold-ms, expected id:positive_milliseconds\n";
        return 2;
      }
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
  uint64_t uid_event_serial = 0;
  uint64_t speech_serial = 0;
  bool anchor_active = false;
  bool anchor_sent = false;
  uint64_t anchor_serial = 0;
  uint64_t anchor_started_ms = 0;
  uint64_t anchor_retry_ms = 0;
  bool motion_ack_denied = false;
  int exit_code = 0;
  bool had_connection = false;
  uint32_t connected_boot_id = 0;

  auto enqueue_speech = [&](const uart::SpeechItem& item) {
    std::string reason;
    if (session.speech_channel_blocked()) reason = "CHANNEL_BLOCKED";
    else if (session.link_state() != uart::LinkState::kConnected) reason = "LINK_NOT_READY";
    if (!reason.empty() || !speech_queue.Enqueue(item, &reason)) {
      std::cout << "SPEECH_REJECT " << item.event_id << " "
                << static_cast<unsigned>(item.audio_id) << " " << reason << "\n" << std::flush;
    } else {
      std::cout << "SPEECH_QUEUED " << item.event_id << " "
                << static_cast<unsigned>(item.audio_id) << "\n" << std::flush;
    }
  };
  auto anchor_fail = [&](const char* reason) {
    anchor_active = false;
    anchor_sent = false;
    have_cmd = false;
    session.SetVelocity(0.0f, 0.0f);
    std::cout << "ANCHOR_FAIL " << reason << "\n" << std::flush;
  };

  while (!g_stop.load()) {
    session.Poll();
    if (require_heading_anchor && session.link_state() == uart::LinkState::kConnected &&
        !session.supports_heading_reference()) {
      std::cout << "ANCHOR_FAIL UNSUPPORTED\n" << std::flush;
      exit_code = 1;
      break;
    }
    if (session.link_state() == uart::LinkState::kConnected && !session.command_enabled() &&
        !session.management_request_pending()) {
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
        enqueue_speech({"uid-" + std::to_string(++uid_event_serial), rfid.card_number});
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

    uart::RequestCompletion completed;
    while (session.PopRequestCompletion(&completed)) {
      if (completed.request_type == uart::MsgType::kSpeakAudio &&
          completed.serial == speech_serial) {
        uart::SpeechItem item;
        if (speech_queue.Finish(&item)) {
          const char* result = CompletionName(completed);
          std::cout << "SPEECH_RESULT " << item.event_id << " "
                    << static_cast<unsigned>(item.audio_id) << " " << result << "\n" << std::flush;
        }
        if (completed.code != uart::RequestCompletionCode::kAck) {
          while (speech_queue.PopQueued(&item)) {
            std::cout << "SPEECH_REJECT " << item.event_id << " "
                      << static_cast<unsigned>(item.audio_id) << " "
                      << CompletionName(completed) << "\n" << std::flush;
          }
        }
      } else if (completed.request_type == uart::MsgType::kHeadingReference &&
                 anchor_active && completed.serial == anchor_serial) {
        anchor_sent = false;
        if (completed.code == uart::RequestCompletionCode::kAck &&
            completed.result == uart::AckResult::kOk) {
          anchor_active = false;
          // Poll may have read ODOM before this ACK. Do not give that cached
          // sample a new stdout timestamp; wait for the next received frame.
          seen_odom_us = session.telemetry().odom_us;
          std::cout << "ANCHOR_DONE\n" << std::flush;
        } else if (completed.code == uart::RequestCompletionCode::kAck &&
                   completed.result == uart::AckResult::kBusy &&
                   clock.NowMs() - anchor_started_ms < 2000) {
          anchor_retry_ms = clock.NowMs() + 100;
        } else {
          anchor_fail(CompletionName(completed));
        }
      } else if (completed.request_type == uart::MsgType::kMotionAction &&
                 completed.code == uart::RequestCompletionCode::kAck &&
                 completed.result != uart::AckResult::kOk) {
        motion_ack_denied = true;
      }
    }

    const bool connected = session.link_state() == uart::LinkState::kConnected;
    if (had_connection && (!connected || session.boot_id() != connected_boot_id)) {
      uart::SpeechItem item;
      while (speech_queue.PopQueued(&item)) {
        std::cout << "SPEECH_REJECT " << item.event_id << " "
                  << static_cast<unsigned>(item.audio_id) << " LINK_LOST\n" << std::flush;
      }
      if (anchor_active) anchor_fail("LINK_LOST");
    }
    had_connection = connected;
    if (connected) connected_boot_id = session.boot_id();

    auto apply_line = [&](const std::string& line) {
      if (line.rfind("speech ", 0) == 0) {
        std::istringstream in(line);
        std::string verb, event_id, extra;
        unsigned id = 0;
        if (!(in >> verb >> event_id >> id) || (in >> extra) || id < 1 || id > 32) {
          std::cout << "SPEECH_REJECT invalid 0 bad_request\n" << std::flush;
        } else {
          enqueue_speech({event_id, static_cast<uint8_t>(id)});
        }
        return;
      }
      if (line == "anchor_heading") {
        have_cmd = false;
        linear = angular = 0.0f;
        if (anchor_active || finite_action != FiniteAction::kNone) {
          std::cout << "ANCHOR_FAIL BUSY\n" << std::flush;
        } else if (!session.supports_heading_reference()) {
          std::cout << "ANCHOR_FAIL UNSUPPORTED\n" << std::flush;
        } else {
          session.SetVelocity(0.0f, 0.0f);
          anchor_active = true;
          anchor_sent = false;
          anchor_started_ms = clock.NowMs();
          anchor_retry_ms = anchor_started_ms;
        }
        return;
      }
      if (line == "stop") {
        if (anchor_active) anchor_fail("CANCELLED");
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
        return;
      }
      if (line == "turn left" || line == "turn right") {
        if (anchor_active) { std::cout << "TURN_FAIL\n" << std::flush; return; }
        motion_ack_denied = false;
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
        return;
      }
      if (line.rfind("forward ", 0) == 0 || line.rfind("backward ", 0) == 0) {
        motion_ack_denied = false;
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
        if (!valid || anchor_active || finite_action != FiniteAction::kNone) {
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
        return;
      }
      float next_v = 0.0f;
      float next_w = 0.0f;
      std::istringstream in(line);
      if (!(in >> next_v >> next_w)) {
        next_v = 0.0f;
        next_w = 0.0f;
      }
      if (anchor_active) { next_v = 0.0f; next_w = 0.0f; }
      linear = next_v;
      angular = next_w;
      have_cmd = true;
      cmd_ms = clock.NowMs();
    };

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
        apply_line(line);
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
      const bool denied = motion_ack_denied;
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

    if (anchor_active) {
      session.SetVelocity(0.0f, 0.0f);
      if (!anchor_sent && now_ms - anchor_started_ms >= 2000) {
        anchor_fail("BUSY");
      } else if (!anchor_sent && now_ms >= anchor_retry_ms &&
                 !session.management_request_pending()) {
        if (session.RequestHeadingReference()) {
          anchor_sent = true;
          anchor_serial = session.last_request_serial();
        }
      }
    }
    if (!anchor_active && finite_action == FiniteAction::kNone &&
        !session.management_request_pending() && !session.speech_channel_blocked()) {
      const uart::SpeechItem* next = speech_queue.Next(now_ms);
      if (next != nullptr && session.RequestSpeech(next->audio_id)) {
        const uart::SpeechItem sent = *next;
        speech_serial = session.last_request_serial();
        speech_queue.MarkSent(now_ms);
        std::cout << "SPEECH_SENT " << sent.event_id << " "
                  << static_cast<unsigned>(sent.audio_id) << "\n" << std::flush;
      }
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
  return exit_code;
}
