/// @file
/// 假下位机：解析上位机发出的帧，并按需构造回复。只在测试中使用。

#ifndef UART_TESTS_MOCK_MCU_H_
#define UART_TESTS_MOCK_MCU_H_

#include <cstdint>
#include <vector>

#include "mock/fake.h"
#include "proto/codec.h"
#include "proto/frame.h"
#include "proto/msg.h"

namespace uart::test {

class FakeMcu {
 public:
  struct Received {
    uint8_t type = 0;
    std::vector<uint8_t> payload;
  };

  void Drain(FakePort& port) {
    const std::vector<uint8_t> bytes = port.tx();
    port.ClearTx();
    rx_.Feed(bytes.data(), bytes.size(), now_us_,
             [this](uint8_t type, const uint8_t* payload, size_t len) {
               Received got;
               got.type = type;
               got.payload.assign(payload, payload + len);
               received_.push_back(std::move(got));
             });
  }

  const std::vector<Received>& received() const { return received_; }
  void ClearReceived() { received_.clear(); }

  size_t CountOf(MsgType type) const {
    size_t n = 0;
    for (const Received& frame : received_) {
      if (frame.type == static_cast<uint8_t>(type)) {
        ++n;
      }
    }
    return n;
  }

  const Received* Last(MsgType type) const {
    for (auto it = received_.rbegin(); it != received_.rend(); ++it) {
      if (it->type == static_cast<uint8_t>(type)) {
        return &*it;
      }
    }
    return nullptr;
  }

  const Reassembler::Stats& stats() const { return rx_.stats(); }

  void SendHelloInfo(FakePort& port, const HelloInfo& msg) {
    uint8_t payload[kSizeHelloInfo] = {};
    EncodeHelloInfo(msg, payload, sizeof(payload));
    Emit(port, MsgType::kHelloInfo, payload, sizeof(payload));
  }

  void SendSystemStatus(FakePort& port, const SystemStatus& msg) {
    uint8_t payload[kSizeSystemStatus] = {};
    EncodeSystemStatus(msg, payload, sizeof(payload));
    Emit(port, MsgType::kSystemStatus, payload, sizeof(payload));
  }

  void SendAck(FakePort& port, const Ack& msg) {
    uint8_t payload[kSizeAck] = {};
    EncodeAck(msg, payload, sizeof(payload));
    Emit(port, MsgType::kAck, payload, sizeof(payload));
  }

  void SendOdom(FakePort& port, const OdomState& msg) {
    uint8_t payload[kSizeOdomState] = {};
    EncodeOdomState(msg, payload, sizeof(payload));
    Emit(port, MsgType::kOdomState, payload, sizeof(payload));
  }

  void SendMotionResult(FakePort& port, const MotionResult& msg) {
    uint8_t payload[kSizeMotionResult] = {};
    EncodeMotionResult(msg, payload, sizeof(payload));
    Emit(port, MsgType::kMotionResult, payload, sizeof(payload));
  }

  void SendRfidCard(FakePort& port, const RfidCard& msg) {
    uint8_t payload[kSizeRfidCard] = {};
    EncodeRfidCard(msg, payload, sizeof(payload));
    Emit(port, MsgType::kRfidCard, payload, sizeof(payload));
  }

  void SendHelloInfoWithVersion(FakePort& port, HelloInfo msg, uint8_t version) {
    msg.protocol_version = version;
    SendHelloInfo(port, msg);
  }

  void SendRaw(FakePort& port, const std::vector<uint8_t>& bytes) { port.PushRx(bytes); }

  void set_now_us(uint64_t us) { now_us_ = us; }

 private:
  void Emit(FakePort& port, MsgType type, const uint8_t* payload, size_t len) {
    uint8_t frame[kMaxFrameSize] = {};
    const size_t frame_len =
        EncodeFrame(static_cast<uint8_t>(type), payload, len, frame, sizeof(frame));
    port.PushRx(frame, frame_len);
  }

  Reassembler rx_;
  std::vector<Received> received_;
  uint64_t now_us_ = 0;
};

}  // namespace uart::test

#endif  // UART_TESTS_MOCK_MCU_H_
