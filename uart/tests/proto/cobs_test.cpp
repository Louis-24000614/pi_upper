/// @file
/// COBS 编解码单元测试。重点在 0xFF 满块边界和对非法输入的拒绝——这两处写错时
/// 短帧照样能跑通，只有长帧或丢包错位才暴露。

#include "proto/cobs.h"

#include <cstring>
#include <vector>

#include "check.h"

namespace {

using uart::CobsDecode;
using uart::CobsEncode;
using uart::CobsEncodeBound;

/// 编码后解码必须还原原始数据，且编码结果不含 0x00（否则无法用作帧边界）。
void ExpectRoundTrip(const std::vector<uint8_t>& raw) {
  std::vector<uint8_t> encoded(CobsEncodeBound(raw.size()));
  const size_t encoded_len = CobsEncode(raw.data(), raw.size(), encoded.data(), encoded.size());
  CHECK(encoded_len > 0);
  for (size_t i = 0; i < encoded_len; ++i) {
    CHECK(encoded[i] != 0);
  }

  std::vector<uint8_t> decoded(raw.size() + 1);
  const size_t decoded_len = CobsDecode(encoded.data(), encoded_len, decoded.data(), decoded.size());
  CHECK(decoded_len == raw.size());
  if (decoded_len == raw.size() && !raw.empty()) {
    CHECK(std::memcmp(decoded.data(), raw.data(), raw.size()) == 0);
  }
}

void TestRoundTrips() {
  ExpectRoundTrip({});
  ExpectRoundTrip({0x01});
  ExpectRoundTrip({0x00});
  ExpectRoundTrip({0x00, 0x00, 0x00});
  ExpectRoundTrip({0x11, 0x22, 0x00, 0x33});
  ExpectRoundTrip({0x00, 0x11, 0x00, 0x00, 0x22, 0x00});

  // 253 / 254 / 255 个非零字节，跨越 0xFF 满块边界。
  for (size_t len : {253u, 254u, 255u}) {
    ExpectRoundTrip(std::vector<uint8_t>(len, 0xAB));
  }

  // 满块紧跟一个零字节，是最容易写错的组合。
  std::vector<uint8_t> block(254, 0xAB);
  block.push_back(0x00);
  block.push_back(0xCD);
  ExpectRoundTrip(block);
}

/// 恰好 254 个非零字节：满块 flush 之后还会补一个空块码字，
/// 即 0xFF + 254 字节数据 + 0x01，共 256 字节。
///
/// 这里有两种流派——省掉末尾空块（255 字节）的实现也能自洽地往返，
/// 但必须与固件一致才能互通。已对照下位机 UartProtocol_CobsEncode 确认为本形式。
void TestFullBlockEncoding() {
  const std::vector<uint8_t> raw(254, 0xAB);
  std::vector<uint8_t> encoded(CobsEncodeBound(raw.size()));
  const size_t len = CobsEncode(raw.data(), raw.size(), encoded.data(), encoded.size());
  CHECK(len == 256);
  CHECK(encoded[0] == 0xFF);
  CHECK(encoded[255] == 0x01);
}

/// 单个零字节编码为 01 01：两个空块。
void TestSingleZeroEncoding() {
  const uint8_t raw[] = {0x00};
  uint8_t encoded[4] = {};
  const size_t len = CobsEncode(raw, sizeof(raw), encoded, sizeof(encoded));
  CHECK(len == 2);
  CHECK(encoded[0] == 0x01);
  CHECK(encoded[1] == 0x01);
}

void TestEncodeRejectsSmallBuffer() {
  const uint8_t raw[] = {0x11, 0x22, 0x33};
  uint8_t encoded[3] = {};  // 需要 4 字节
  CHECK(CobsEncode(raw, sizeof(raw), encoded, sizeof(encoded)) == 0);
}

/// 非法输入必须被拒绝而不是解出垃圾数据：丢包错位时这是唯一的防线。
void TestDecodeRejectsInvalid() {
  uint8_t out[64] = {};

  // 编码数据内部出现分隔符。
  const uint8_t embedded_zero[] = {0x03, 0x11, 0x00};
  CHECK(CobsDecode(embedded_zero, sizeof(embedded_zero), out, sizeof(out)) == 0);

  // 码字声明的长度跨过数据末尾。
  const uint8_t truncated[] = {0x05, 0x11, 0x22};
  CHECK(CobsDecode(truncated, sizeof(truncated), out, sizeof(out)) == 0);

  // 目标缓冲不足。
  const uint8_t valid[] = {0x04, 0x11, 0x22, 0x33};
  CHECK(CobsDecode(valid, sizeof(valid), out, 2) == 0);
}

}  // namespace

int main() {
  TestRoundTrips();
  TestFullBlockEncoding();
  TestSingleZeroEncoding();
  TestEncodeRejectsSmallBuffer();
  TestDecodeRejectsInvalid();
  return uart::test::Finish("cobs");
}
