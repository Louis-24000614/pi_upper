/// @file
/// CRC-8/ATM 测试。核心是协议文档第 3 节给出的检查值，它能同时锁住多项式、初值、
/// 反射方向和最终异或——这四个参数任何一个搞错，检查值都不会是 0xF4。

#include "proto/crc8.h"

#include <cstring>

#include "check.h"

namespace {

using uart::Crc8;
using uart::Crc8Update;

uint8_t Crc8Str(const char* s) {
  return Crc8(reinterpret_cast<const uint8_t*>(s), std::strlen(s));
}

/// CRC-8/ATM 的标准检查值。
void TestCheckValue() { CHECK(Crc8Str("123456789") == 0xF4); }

void TestEmpty() {
  CHECK(Crc8(nullptr, 0) == 0x00);
  const uint8_t data[1] = {0};
  CHECK(Crc8(data, 0) == 0x00);
}

/// 逐字节推进与一次性计算必须一致，收帧状态机边收边算就依赖这一点。
void TestIncrementalMatchesBulk() {
  const uint8_t data[] = {0x01, 0x01, 0x02, 0xFF, 0x00, 0x55, 0xAA, 0x7F};
  uint8_t crc = 0;
  for (uint8_t byte : data) {
    crc = Crc8Update(crc, byte);
  }
  CHECK(crc == Crc8(data, sizeof(data)));
}

/// 文档黄金帧里的 CRC 覆盖范围是 TYPE + LENGTH + PAYLOAD，这里单独验算这一段。
void TestProtocolVectors() {
  const uint8_t hello[] = {0x01, 0x01, 0x02};
  CHECK(Crc8(hello, sizeof(hello)) == 0x70);

  const uint8_t cmd_vel[] = {0x12, 0x0C, 0x78, 0x56, 0x34, 0x12, 0x00,
                             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  CHECK(Crc8(cmd_vel, sizeof(cmd_vel)) == 0xCD);
}

void TestDetectsSingleBitFlip() {
  const uint8_t good[] = {0x12, 0x0C, 0x78};
  const uint8_t bad[] = {0x12, 0x0C, 0x79};
  CHECK(Crc8(good, sizeof(good)) != Crc8(bad, sizeof(bad)));
}

}  // namespace

int main() {
  TestCheckValue();
  TestEmpty();
  TestIncrementalMatchesBulk();
  TestProtocolVectors();
  TestDetectsSingleBitFlip();
  return uart::test::Finish("crc8");
}
