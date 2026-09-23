/// @file
/// CRC-8/ATM 测试。核心是协议文档第 3 节给出的检查值。

#include "proto/crc8.h"

#include <cstring>

#include "check.h"

namespace {

using uart::Crc8;
using uart::Crc8Update;

uint8_t Crc8Str(const char* s) {
  return Crc8(reinterpret_cast<const uint8_t*>(s), std::strlen(s));
}

void TestCheckValue() { CHECK(Crc8Str("123456789") == 0xF4); }

void TestEmpty() {
  CHECK(Crc8(nullptr, 0) == 0x00);
  const uint8_t data[1] = {0};
  CHECK(Crc8(data, 0) == 0x00);
}

void TestIncrementalMatchesBulk() {
  const uint8_t data[] = {0x01, 0x01, 0x01, 0xFF, 0x00, 0x55, 0xAA, 0x7F};
  uint8_t crc = 0;
  for (uint8_t byte : data) {
    crc = Crc8Update(crc, byte);
  }
  CHECK(crc == Crc8(data, sizeof(data)));
}

/// 固件黄金帧：HELLO_REQ `55 AA 01 01 01 79`，零速 CMD_VEL `55 AA 12 08 ... 83`。
void TestProtocolVectors() {
  const uint8_t hello[] = {0x01, 0x01, 0x01};
  CHECK(Crc8(hello, sizeof(hello)) == 0x79);

  const uint8_t cmd_vel[] = {0x12, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  CHECK(Crc8(cmd_vel, sizeof(cmd_vel)) == 0x83);
}

void TestDetectsSingleBitFlip() {
  const uint8_t good[] = {0x12, 0x08, 0x00};
  const uint8_t bad[] = {0x12, 0x08, 0x01};
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
