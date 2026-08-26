#include "proto/crc32c.h"

#include <array>

namespace uart {
namespace {

/// 反射式查表法的 256 项表，编译期生成，避免运行时初始化顺序问题。
/// RK3588 有硬件 CRC32C 指令（ARMv8 CRC 扩展），但字节表法在 921600 波特率下
/// 开销可以忽略（最大帧 256 字节），先保持可移植实现，需要时再换。
constexpr std::array<uint32_t, 256> MakeTable() {
  constexpr uint32_t kReflectedPoly = 0x82F63B78u;
  std::array<uint32_t, 256> table{};
  for (uint32_t i = 0; i < 256; ++i) {
    uint32_t crc = i;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 1u) ? ((crc >> 1) ^ kReflectedPoly) : (crc >> 1);
    }
    table[i] = crc;
  }
  return table;
}

constexpr std::array<uint32_t, 256> kTable = MakeTable();

}  // namespace

uint32_t Crc32cUpdate(uint32_t state, const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; ++i) {
    state = kTable[(state ^ data[i]) & 0xFFu] ^ (state >> 8);
  }
  return state;
}

uint32_t Crc32c(const uint8_t* data, size_t len) {
  return Crc32cFinish(Crc32cUpdate(kCrc32cInit, data, len));
}

}  // namespace uart
