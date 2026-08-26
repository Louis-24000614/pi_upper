/// @file
/// 帧层单元测试。核心是 UART_PROTOCOL.md 里的黄金帧——它们是上位机实现与固件实现
/// 交叉校验的唯一硬标准，字节级不一致就说明两端对不上。
///
/// v2 的重点测试对象与 v1 不同：不再有 COBS 和分隔符，改为验证定长字段定帧、
/// payload 里出现 `55 AA` 不误判、以及字节间超时后的重新同步。

#include "proto/frame.h"

#include <cstring>

#include "proto/crc8.h"
#include <vector>

#include "check.h"

namespace {

using uart::EncodeFrame;
using uart::kInterByteTimeoutUs;
using uart::kMaxFrameSize;
using uart::kMaxPayloadSize;
using uart::Reassembler;

/// 文档 6.1：HELLO_REQ，payload 为单字节协议版本 2。
const std::vector<uint8_t> kGoldenHelloReq = {0x55, 0xAA, 0x01, 0x01, 0x02, 0x70};

/// 文档 6.5：零速 CMD_VEL，token 0x12345678，v 与 ω 均为 0.0f。
const std::vector<uint8_t> kGoldenCmdVel = {0x55, 0xAA, 0x12, 0x0C, 0x78, 0x56, 0x34, 0x12, 0x00,
                                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xCD};

/// 文档第 9 节：把 token 首字节从 0x78 改成 0x79 却保留原 CRC 0xCD。
std::vector<uint8_t> MakeGoldenCrcError() {
  std::vector<uint8_t> bad = kGoldenCmdVel;
  bad[4] = 0x79;
  return bad;
}

/// 收帧结果的记录器，省掉每个用例都写一遍 lambda 捕获。
struct Sink {
  int calls = 0;
  uint8_t msg_type = 0;
  std::vector<uint8_t> payload;

  Reassembler::FrameHandler Handler() {
    return [this](uint8_t type, const uint8_t* data, size_t len) {
      ++calls;
      msg_type = type;
      payload.assign(data, data + len);
    };
  }
};

void ExpectBytes(const uint8_t* got, size_t got_len, const std::vector<uint8_t>& want,
                 const char* what) {
  CHECK(got_len == want.size());
  if (got_len == want.size() && std::memcmp(got, want.data(), want.size()) == 0) {
    return;
  }
  ++uart::test::g_failures;
  std::fprintf(stderr, "FAIL %s\n  got:  %s\n  want: %s\n", what,
               uart::test::Hex(got, got_len).c_str(),
               uart::test::Hex(want.data(), want.size()).c_str());
}

/// 编码方向：装配出的字节流必须与黄金帧完全一致。
void TestEncodeGoldenVectors() {
  uint8_t buf[kMaxFrameSize] = {};

  const uint8_t hello_payload[1] = {0x02};
  size_t len = EncodeFrame(0x01, hello_payload, sizeof(hello_payload), buf, sizeof(buf));
  ExpectBytes(buf, len, kGoldenHelloReq, "encode HELLO_REQ");

  // CMD_VEL payload：token(u32) + linear_x(f32) + angular_z(f32)，均小端。
  const uint8_t payload[12] = {0x78, 0x56, 0x34, 0x12, 0, 0, 0, 0, 0, 0, 0, 0};
  len = EncodeFrame(0x12, payload, sizeof(payload), buf, sizeof(buf));
  ExpectBytes(buf, len, kGoldenCmdVel, "encode zero CMD_VEL");
}

/// 解码方向：黄金帧喂进收帧状态机，应解出正确的类型与 payload。
void TestDecodeGoldenVectors() {
  Reassembler rx;
  Sink sink;
  const auto handler = sink.Handler();

  rx.Feed(kGoldenHelloReq.data(), kGoldenHelloReq.size(), 0, handler);
  CHECK(sink.calls == 1);
  CHECK(sink.msg_type == 0x01);
  CHECK(sink.payload.size() == 1);
  if (sink.payload.size() == 1) {
    CHECK(sink.payload[0] == uart::kProtocolVersion);
  }

  rx.Feed(kGoldenCmdVel.data(), kGoldenCmdVel.size(), 0, handler);
  CHECK(sink.calls == 2);
  CHECK(sink.msg_type == 0x12);
  CHECK(sink.payload.size() == 12);
  if (sink.payload.size() == 12) {
    CHECK(sink.payload[0] == 0x78 && sink.payload[3] == 0x12);
  }
  CHECK(rx.stats().frames == 2);
  CHECK(rx.stats().crc_errors == 0);
}

/// 坏 CRC 必须计入 crc_errors 且不产生任何回调。
void TestGoldenCrcErrorIsRejected() {
  const std::vector<uint8_t> bad = MakeGoldenCrcError();
  Reassembler rx;
  Sink sink;
  rx.Feed(bad.data(), bad.size(), 0, sink.Handler());
  CHECK(sink.calls == 0);
  CHECK(rx.stats().crc_errors == 1);
  CHECK(rx.stats().frames == 0);
}

/// 逐字节喂入必须与整块喂入等价：真实串口读到的分片边界是任意的。
void TestByteAtATimeFeed() {
  Reassembler rx;
  Sink sink;
  const auto handler = sink.Handler();
  for (uint8_t byte : kGoldenCmdVel) {
    rx.Feed(&byte, 1, 0, handler);
  }
  CHECK(sink.calls == 1);
  CHECK(rx.stats().frames == 1);
}

/// 两帧粘在一起、以及帧被切成任意两段，都必须能正确解出。
void TestSplitAndConcatenated() {
  std::vector<uint8_t> two = kGoldenHelloReq;
  two.insert(two.end(), kGoldenCmdVel.begin(), kGoldenCmdVel.end());

  Reassembler rx;
  Sink sink;
  rx.Feed(two.data(), two.size(), 0, sink.Handler());
  CHECK(sink.calls == 2);

  for (size_t cut = 1; cut < two.size(); ++cut) {
    Reassembler split_rx;
    Sink split_sink;
    const auto handler = split_sink.Handler();
    split_rx.Feed(two.data(), cut, 0, handler);
    split_rx.Feed(two.data() + cut, two.size() - cut, 0, handler);
    CHECK(split_sink.calls == 2);
  }
}

/// v2 不用转义，payload 里可以出现同步字。接收端必须严格按 LENGTH 取数据，
/// 不能把 payload 内部的 55 AA 当成新帧的开始。
void TestSyncBytesInsidePayload() {
  const uint8_t payload[] = {0x55, 0xAA, 0x55, 0xAA, 0x00, 0x55, 0xAA};
  uint8_t buf[kMaxFrameSize] = {};
  const size_t len = EncodeFrame(0x90, payload, sizeof(payload), buf, sizeof(buf));
  CHECK(len == sizeof(payload) + uart::kFrameOverhead);

  Reassembler rx;
  Sink sink;
  rx.Feed(buf, len, 0, sink.Handler());
  CHECK(sink.calls == 1);
  CHECK(sink.payload.size() == sizeof(payload));
  if (sink.payload.size() == sizeof(payload)) {
    CHECK(std::memcmp(sink.payload.data(), payload, sizeof(payload)) == 0);
  }
}

/// 同步字前的垃圾数据要被跳过；连续的 0x55 不能让状态机错过真正的 55 AA。
void TestResyncFromGarbage() {
  Reassembler rx;
  Sink sink;
  const auto handler = sink.Handler();

  const uint8_t garbage[] = {0x00, 0xFF, 0x55, 0x55, 0x55};
  rx.Feed(garbage, sizeof(garbage), 0, handler);
  CHECK(sink.calls == 0);

  // 上面最后一个 0x55 已经进入等待第二同步字的状态，紧接一个 AA 就应开始收帧。
  // 因此这里只喂黄金帧去掉首字节 0x55 的剩余部分。
  rx.Feed(kGoldenHelloReq.data() + 1, kGoldenHelloReq.size() - 1, 0, handler);
  CHECK(sink.calls == 1);
  CHECK(sink.msg_type == 0x01);
}

/// 帧收到一半断流超过字节间超时：残帧必须丢弃，否则会与后续字节拼出错位的"合法"帧。
void TestInterByteTimeout() {
  Reassembler rx;
  Sink sink;
  const auto handler = sink.Handler();

  // 只喂半帧。
  rx.Feed(kGoldenCmdVel.data(), 6, 1000, handler);
  CHECK(sink.calls == 0);

  // 超时之后补上剩余字节：残帧已被丢弃，这些字节凑不出一帧。
  rx.Feed(kGoldenCmdVel.data() + 6, kGoldenCmdVel.size() - 6, 1000 + kInterByteTimeoutUs + 1,
          handler);
  CHECK(sink.calls == 0);
  CHECK(rx.stats().timeouts == 1);

  // 超时不应破坏状态机，之后的完整帧仍要能收。
  rx.Feed(kGoldenHelloReq.data(), kGoldenHelloReq.size(), 2000000, handler);
  CHECK(sink.calls == 1);
}

/// 超时窗口内的分片是正常现象，不能误丢。
void TestNoTimeoutWithinWindow() {
  Reassembler rx;
  Sink sink;
  const auto handler = sink.Handler();
  rx.Feed(kGoldenCmdVel.data(), 6, 1000, handler);
  rx.Feed(kGoldenCmdVel.data() + 6, kGoldenCmdVel.size() - 6, 1000 + kInterByteTimeoutUs, handler);
  CHECK(sink.calls == 1);
  CHECK(rx.stats().timeouts == 0);
}

/// 所有合法 payload 长度的编解码往返，含 0 和上限 128。
void TestRoundTripAllLengths() {
  uint8_t buf[kMaxFrameSize] = {};
  for (size_t payload_len = 0; payload_len <= kMaxPayloadSize; ++payload_len) {
    std::vector<uint8_t> payload(payload_len);
    for (size_t i = 0; i < payload_len; ++i) {
      payload[i] = static_cast<uint8_t>(i);
    }

    const size_t len = EncodeFrame(0x90, payload.data(), payload_len, buf, sizeof(buf));
    CHECK(len == payload_len + uart::kFrameOverhead);

    Reassembler rx;
    Sink sink;
    rx.Feed(buf, len, 0, sink.Handler());
    CHECK(sink.calls == 1);
    CHECK(sink.msg_type == 0x90);
    CHECK(sink.payload.size() == payload_len);
    if (sink.payload.size() == payload_len && payload_len > 0) {
      CHECK(std::memcmp(sink.payload.data(), payload.data(), payload_len) == 0);
    }
  }
}

void TestEncodeRejectsBadArguments() {
  uint8_t buf[kMaxFrameSize] = {};
  const std::vector<uint8_t> payload(kMaxPayloadSize + 1, 0xAB);
  // payload 超过 128 字节。
  CHECK(EncodeFrame(0x90, payload.data(), payload.size(), buf, sizeof(buf)) == 0);
  // 目标缓冲不足。
  CHECK(EncodeFrame(0x90, payload.data(), 8, buf, 8) == 0);
  // 长度非 0 时 payload 不能为空指针。
  CHECK(EncodeFrame(0x90, nullptr, 4, buf, sizeof(buf)) == 0);
}

/// LENGTH 字段超过上限：计一次溢出并重新找同步字，不能按超长长度去读后续字节。
void TestOversizeLengthField() {
  Reassembler rx;
  Sink sink;
  const auto handler = sink.Handler();

  const uint8_t bad[] = {0x55, 0xAA, 0x90, 0xFF};
  rx.Feed(bad, sizeof(bad), 0, handler);
  CHECK(rx.stats().overflows == 1);
  CHECK(sink.calls == 0);

  rx.Feed(kGoldenHelloReq.data(), kGoldenHelloReq.size(), 0, handler);
  CHECK(sink.calls == 1);
  CHECK(rx.stats().frames == 1);
}

/// 坏帧的最后一个字节恰好是 0x55 时，紧随其后的帧仍要能收到。
void TestResyncWhenBadCrcByteIsSync() {
  Reassembler rx;
  Sink sink;
  const auto handler = sink.Handler();

  // 构造一帧 CRC 位置为 0x55 的坏帧，紧接一个完整的黄金帧。
  std::vector<uint8_t> stream = {0x55, 0xAA, 0x11, 0x00, 0x55};
  CHECK(uart::Crc8(stream.data() + 2, 2) != 0x55);
  stream.insert(stream.end(), kGoldenHelloReq.begin() + 1, kGoldenHelloReq.end());

  rx.Feed(stream.data(), stream.size(), 0, handler);
  CHECK(rx.stats().crc_errors == 1);
  CHECK(sink.calls == 1);
  CHECK(sink.msg_type == 0x01);
}

/// Reset 丢弃残帧但保留统计。
void TestReset() {
  Reassembler rx;
  Sink sink;
  const auto handler = sink.Handler();

  rx.Feed(kGoldenHelloReq.data(), kGoldenHelloReq.size(), 0, handler);
  rx.Feed(kGoldenCmdVel.data(), 6, 0, handler);
  rx.Reset();
  rx.Feed(kGoldenCmdVel.data() + 6, kGoldenCmdVel.size() - 6, 0, handler);
  CHECK(sink.calls == 1);
  CHECK(rx.stats().frames == 1);
}

}  // namespace

int main() {
  TestEncodeGoldenVectors();
  TestDecodeGoldenVectors();
  TestGoldenCrcErrorIsRejected();
  TestByteAtATimeFeed();
  TestSplitAndConcatenated();
  TestSyncBytesInsidePayload();
  TestResyncFromGarbage();
  TestInterByteTimeout();
  TestNoTimeoutWithinWindow();
  TestRoundTripAllLengths();
  TestEncodeRejectsBadArguments();
  TestOversizeLengthField();
  TestResyncWhenBadCrcByteIsSync();
  TestReset();
  return uart::test::Finish("frame");
}
