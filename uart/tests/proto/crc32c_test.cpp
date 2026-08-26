/// @file
/// CRC-32C 单元测试。核心是协议文档给出的校验值，用来确认多项式和反射方向没搞错。

#include "proto/crc32c.h"

#include <cstring>

#include "check.h"

namespace {

using uart::Crc32c;
using uart::Crc32cFinish;
using uart::Crc32cUpdate;
using uart::kCrc32cInit;

/// 协议文档给定：校验串 "123456789" 的 CRC-32C 为 0xE3069283。
/// 这一条能同时排除"用错多项式"（比如误用 CRC-32 的 0xEDB88320）和"反射方向搞反"。
void TestCheckString() {
  const char* s = "123456789";
  CHECK(Crc32c(reinterpret_cast<const uint8_t*>(s), std::strlen(s)) == 0xE3069283u);
}

void TestEmptyInput() {
  CHECK(Crc32c(nullptr, 0) == 0u);
}

/// 分段计算必须与一次性计算等价，否则帧头与 payload 分开算 CRC 的路径会出错。
void TestSegmentedMatchesWhole() {
  const uint8_t data[] = {0x5A, 0xA5, 0x01, 0x12, 0x00, 0x00, 0x02, 0x00, 0x0C, 0x00};
  const uint32_t whole = Crc32c(data, sizeof(data));

  for (size_t split = 0; split <= sizeof(data); ++split) {
    uint32_t state = Crc32cUpdate(kCrc32cInit, data, split);
    state = Crc32cUpdate(state, data + split, sizeof(data) - split);
    CHECK(Crc32cFinish(state) == whole);
  }
}

/// 单比特翻转必须改变 CRC，确认查表没有退化成常量。
void TestBitFlipChangesCrc() {
  uint8_t data[16] = {};
  const uint32_t base = Crc32c(data, sizeof(data));
  data[7] ^= 0x01;
  CHECK(Crc32c(data, sizeof(data)) != base);
}

}  // namespace

int main() {
  TestCheckString();
  TestEmptyInput();
  TestSegmentedMatchesWhole();
  TestBitFlipChangesCrc();
  return uart::test::Finish("crc32c");
}
