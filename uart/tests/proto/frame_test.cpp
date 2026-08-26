/// @file
/// 帧层单元测试。核心是 UART_PROTOCOL.md 第 10 节的三条黄金测试向量——它们是
/// 上位机实现与固件实现交叉校验的唯一硬标准，字节级不一致就说明两端对不上。

#include "proto/frame.h"

#include <cstring>
#include <vector>

#include "check.h"

namespace {

using uart::DecodeRawFrame;
using uart::EncodeFrame;
using uart::FrameError;
using uart::Header;
using uart::kMaxEncodedSize;
using uart::kMaxPayloadSize;
using uart::Reassembler;

/// 协议 10.1：HELLO_REQ，版本 1，消息 0x01，序号 1，时间戳 0，空 payload。
const std::vector<uint8_t> kGoldenHelloReq = {0x05, 0x5A, 0xA5, 0x01, 0x01, 0x01, 0x02, 0x01,
                                             0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
                                             0x01, 0x01, 0x05, 0xD0, 0xB5, 0xF9, 0xAB, 0x00};

/// 协议 10.2：零速 CMD_VEL，消息 0x12，序号 2，时间戳 0，token 0x12345678，v 与 ω 均为 0.0f。
const std::vector<uint8_t> kGoldenCmdVel = {
    0x05, 0x5A, 0xA5, 0x01, 0x12, 0x01, 0x02, 0x02, 0x02, 0x0C, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x05, 0x78, 0x56, 0x34, 0x12, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x05, 0x17, 0x78, 0x7C, 0x17, 0x00};

/// 协议 10.3：在 10.2 的已编码帧上把消息类型改成 0x13 却保留旧 CRC。
/// 接收端必须判为 CRC 错误，且绝不能执行 RESET_ODOM。
std::vector<uint8_t> MakeGoldenCrcError() {
  std::vector<uint8_t> bad = kGoldenCmdVel;
  bad[4] = 0x13;
  return bad;
}

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

/// 编码方向：装配出的字节流必须与黄金向量完全一致。
void TestEncodeGoldenVectors() {
  uint8_t buf[kMaxEncodedSize] = {};

  Header hello;
  hello.msg_type = 0x01;
  hello.sequence = 1;
  size_t len = EncodeFrame(hello, nullptr, 0, buf, sizeof(buf));
  ExpectBytes(buf, len, kGoldenHelloReq, "encode HELLO_REQ");

  // CMD_VEL payload：token(u32) + linear_x(f32) + angular_z(f32)，均小端。
  const uint8_t payload[12] = {0x78, 0x56, 0x34, 0x12, 0, 0, 0, 0, 0, 0, 0, 0};
  Header cmd;
  cmd.msg_type = 0x12;
  cmd.sequence = 2;
  len = EncodeFrame(cmd, payload, sizeof(payload), buf, sizeof(buf));
  ExpectBytes(buf, len, kGoldenCmdVel, "encode zero CMD_VEL");
}

/// 解码方向：黄金向量喂进收帧状态机，应解出正确的帧头与 payload。
void TestDecodeGoldenVectors() {
  Reassembler rx;
  int calls = 0;
  Header seen;
  std::vector<uint8_t> seen_payload;

  const auto handler = [&](const Header& header, const uint8_t* payload, size_t payload_len) {
    ++calls;
    seen = header;
    seen_payload.assign(payload, payload + payload_len);
  };

  rx.Feed(kGoldenHelloReq.data(), kGoldenHelloReq.size(), handler);
  CHECK(calls == 1);
  CHECK(seen.msg_type == 0x01);
  CHECK(seen.version == 1);
  CHECK(seen.sequence == 1);
  CHECK(seen.payload_length == 0);
  CHECK(seen.timestamp_us == 0);

  rx.Feed(kGoldenCmdVel.data(), kGoldenCmdVel.size(), handler);
  CHECK(calls == 2);
  CHECK(seen.msg_type == 0x12);
  CHECK(seen.sequence == 2);
  CHECK(seen.payload_length == 12);
  CHECK(seen_payload.size() == 12);
  if (seen_payload.size() == 12) {
    CHECK(seen_payload[0] == 0x78 && seen_payload[3] == 0x12);
  }
  CHECK(rx.stats().frames == 2);
  CHECK(rx.stats().crc_errors == 0);
  CHECK(rx.stats().format_errors == 0);
}

/// 协议 10.3：坏 CRC 必须计入 crc_errors 且不产生任何回调。
void TestGoldenCrcErrorIsRejected() {
  const std::vector<uint8_t> bad = MakeGoldenCrcError();
  Reassembler rx;
  int calls = 0;
  rx.Feed(bad.data(), bad.size(), [&](const Header&, const uint8_t*, size_t) { ++calls; });
  CHECK(calls == 0);
  CHECK(rx.stats().crc_errors == 1);
  CHECK(rx.stats().frames == 0);
  CHECK(rx.stats().format_errors == 0);
}

/// 逐字节喂入必须与整块喂入等价：真实串口读到的分片边界是任意的。
void TestByteAtATimeFeed() {
  Reassembler rx;
  int calls = 0;
  const auto handler = [&](const Header&, const uint8_t*, size_t) { ++calls; };
  for (uint8_t byte : kGoldenCmdVel) {
    rx.Feed(&byte, 1, handler);
  }
  CHECK(calls == 1);
  CHECK(rx.stats().frames == 1);
}

/// 任意长度 payload 的编解码往返，覆盖到 kMaxPayloadSize 上限。
void TestRoundTripAllLengths() {
  uint8_t buf[kMaxEncodedSize] = {};
  for (size_t payload_len = 0; payload_len <= kMaxPayloadSize; ++payload_len) {
    std::vector<uint8_t> payload(payload_len);
    for (size_t i = 0; i < payload_len; ++i) {
      // 刻意掺入 0x00，让 COBS 在每个长度上都真正干活。
      payload[i] = static_cast<uint8_t>(i % 3 == 0 ? 0 : i);
    }

    Header header;
    header.msg_type = 0x90;
    header.sequence = static_cast<uint16_t>(payload_len);
    header.timestamp_us = 0x0102030405060708ull;
    const size_t len = EncodeFrame(header, payload.data(), payload_len, buf, sizeof(buf));
    CHECK(len > 0);

    Reassembler rx;
    int calls = 0;
    rx.Feed(buf, len, [&](const Header& got, const uint8_t* got_payload, size_t got_len) {
      ++calls;
      CHECK(got.msg_type == 0x90);
      CHECK(got.sequence == payload_len);
      CHECK(got.timestamp_us == 0x0102030405060708ull);
      CHECK(got_len == payload_len);
      if (got_len == payload_len && payload_len > 0) {
        CHECK(std::memcmp(got_payload, payload.data(), payload_len) == 0);
      }
    });
    CHECK(calls == 1);
  }
}

void TestEncodeRejectsOversizePayload() {
  uint8_t buf[kMaxEncodedSize] = {};
  const std::vector<uint8_t> payload(kMaxPayloadSize + 1, 0xAB);
  Header header;
  header.msg_type = 0x90;
  CHECK(EncodeFrame(header, payload.data(), payload.size(), buf, sizeof(buf)) == 0);
}

/// 帧头各字段的非法值分别归入哪一类错误。
void TestDecodeRawFrameErrors() {
  uint8_t buf[kMaxEncodedSize] = {};
  Header header;
  header.msg_type = 0x01;
  header.sequence = 1;
  const size_t encoded_len = EncodeFrame(header, nullptr, 0, buf, sizeof(buf));
  CHECK(encoded_len > 0);

  // 从已编码帧还原出原始帧，便于逐字段做破坏性测试。
  uint8_t raw[uart::kMaxDecodedSize] = {};
  const size_t raw_len = uart::CobsDecode(buf, encoded_len - 1, raw, sizeof(raw));
  CHECK(raw_len == uart::kHeaderSize + uart::kCrcSize);

  Header out;
  const uint8_t* payload = nullptr;
  CHECK(DecodeRawFrame(raw, raw_len, &out, &payload) == FrameError::kOk);

  // 长度不足。
  CHECK(DecodeRawFrame(raw, uart::kHeaderSize, &out, &payload) == FrameError::kLength);

  // 魔数错误。
  uint8_t broken[uart::kMaxDecodedSize] = {};
  std::memcpy(broken, raw, raw_len);
  broken[0] = 0x00;
  CHECK(DecodeRawFrame(broken, raw_len, &out, &payload) == FrameError::kMagic);

  // reserved 非 0。
  std::memcpy(broken, raw, raw_len);
  broken[5] = 0x01;
  CHECK(DecodeRawFrame(broken, raw_len, &out, &payload) == FrameError::kReserved);

  // payload_length 与实际长度不符。
  std::memcpy(broken, raw, raw_len);
  broken[8] = 0x04;
  CHECK(DecodeRawFrame(broken, raw_len, &out, &payload) == FrameError::kLength);

  // 版本不匹配不由帧层拒绝，需要读出来交给会话层判断。
  std::memcpy(broken, raw, raw_len);
  broken[2] = 0x02;
  CHECK(DecodeRawFrame(broken, raw_len, &out, &payload) == FrameError::kCrc);
}

/// 连续分隔符与空帧不应计入错误，这是重新同步时的正常现象。
void TestEmptyFramesIgnored() {
  Reassembler rx;
  int calls = 0;
  const uint8_t zeros[] = {0x00, 0x00, 0x00};
  rx.Feed(zeros, sizeof(zeros), [&](const Header&, const uint8_t*, size_t) { ++calls; });
  CHECK(calls == 0);
  CHECK(rx.stats().format_errors == 0);
  CHECK(rx.stats().crc_errors == 0);

  // 空帧之后仍应能正常收帧。
  rx.Feed(kGoldenHelloReq.data(), kGoldenHelloReq.size(),
          [&](const Header&, const uint8_t*, size_t) { ++calls; });
  CHECK(calls == 1);
}

/// 超长垃圾数据应计一次溢出，并在下一个分隔符处重新同步。
void TestOverflowResync() {
  Reassembler rx;
  int calls = 0;
  const auto handler = [&](const Header&, const uint8_t*, size_t) { ++calls; };

  const std::vector<uint8_t> garbage(kMaxEncodedSize + 64, 0xAB);
  rx.Feed(garbage.data(), garbage.size(), handler);
  CHECK(rx.stats().overflows == 1);
  CHECK(calls == 0);

  const uint8_t delimiter = 0x00;
  rx.Feed(&delimiter, 1, handler);
  rx.Feed(kGoldenHelloReq.data(), kGoldenHelloReq.size(), handler);
  CHECK(calls == 1);
  CHECK(rx.stats().frames == 1);
}

}  // namespace

int main() {
  TestEncodeGoldenVectors();
  TestDecodeGoldenVectors();
  TestGoldenCrcErrorIsRejected();
  TestByteAtATimeFeed();
  TestRoundTripAllLengths();
  TestEncodeRejectsOversizePayload();
  TestDecodeRawFrameErrors();
  TestEmptyFramesIgnored();
  TestOverflowResync();
  return uart::test::Finish("frame");
}
