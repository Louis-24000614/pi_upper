/// @file
/// 假下位机：解析上位机发出的帧，并按需构造回复。
///
/// 只在测试中使用。它让会话层的测试能像真实联调一样"一问一答"，而不是把预先
/// 拼好的字节流硬塞进去——ARM 流程、token 生命周期这类逻辑只有在交互中才暴露。

#ifndef UART_TESTS_MOCK_MCU_H_
#define UART_TESTS_MOCK_MCU_H_

#include <cstdint>
#include <vector>

#include "proto/codec.h"
#include "mock/fake.h"
#include "proto/frame.h"
#include "proto/msg.h"

namespace uart::test {

/// 假下位机。不实现任何安全逻辑，只做协议层面的收发，具体回什么由测试决定。
class FakeMcu {
 public:
  /// 从上位机收到的一帧。v2 的帧头只有类型和长度，没有序号与时间戳。
  struct Received {
    uint8_t type = 0;
    std::vector<uint8_t> payload;
  };

  /// 解析并清空 @p port 的发送缓冲。
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

  /// 收到的指定类型帧数。
  size_t CountOf(MsgType type) const {
    size_t n = 0;
    for (const Received& frame : received_) {
      if (frame.type == static_cast<uint8_t>(type)) {
        ++n;
      }
    }
    return n;
  }

  /// 最后一帧指定类型，没有则返回 nullptr。
  const Received* Last(MsgType type) const {
    for (auto it = received_.rbegin(); it != received_.rend(); ++it) {
      if (it->type == static_cast<uint8_t>(type)) {
        return &*it;
      }
    }
    return nullptr;
  }

  /// 收帧统计，用于验证上位机发出的帧本身合法（不该有 CRC 或格式错误）。
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

  void SendTimeSyncResp(FakePort& port, const TimeSyncResp& msg) {
    uint8_t payload[kSizeTimeSyncResp] = {};
    EncodeTimeSyncResp(msg, payload, sizeof(payload));
    Emit(port, MsgType::kTimeSyncResp, payload, sizeof(payload));
  }

  void SendOdom(FakePort& port, const OdomState& msg) {
    uint8_t payload[kSizeOdomState] = {};
    EncodeOdomState(msg, payload, sizeof(payload));
    Emit(port, MsgType::kOdomState, payload, sizeof(payload));
  }

  void SendFault(FakePort& port, const FaultEvent& msg) {
    uint8_t payload[kSizeFaultEvent] = {};
    EncodeFaultEvent(msg, payload, sizeof(payload));
    Emit(port, MsgType::kFaultEvent, payload, sizeof(payload));
  }

  /// 用指定协议版本发一帧 HELLO_INFO，用于测试版本不兼容的处理。
  /// v2 的版本号在 HELLO_INFO 的 payload 里，不再在帧头。
  void SendHelloInfoWithVersion(FakePort& port, HelloInfo msg, uint8_t version) {
    msg.protocol_version = version;
    SendHelloInfo(port, msg);
  }

  /// 直接注入任意字节，用于构造坏帧。
  void SendRaw(FakePort& port, const std::vector<uint8_t>& bytes) { port.PushRx(bytes); }

  /// 假下位机自己的时间戳，测试里可以推进它来模拟 MCU 时钟。
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
