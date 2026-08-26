#include "proto/crc8.h"

namespace uart {

uint8_t Crc8Update(uint8_t crc, uint8_t byte) {
  crc ^= byte;
  for (int bit = 0; bit < 8; ++bit) {
    // 不反射实现：从高位开始移位，进位时异或多项式。
    crc = (crc & 0x80u) ? static_cast<uint8_t>((crc << 1) ^ 0x07u)
                        : static_cast<uint8_t>(crc << 1);
  }
  return crc;
}

uint8_t Crc8(const uint8_t* data, size_t len) {
  uint8_t crc = 0;
  for (size_t i = 0; i < len; ++i) {
    crc = Crc8Update(crc, data[i]);
  }
  return crc;
}

}  // namespace uart
