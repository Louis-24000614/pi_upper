/// @file
/// 小端序的逐字节读写辅助。模块内部使用，不对外暴露。
///
/// 协议要求所有多字节整数按小端序逐字节序列化，浮点为 IEEE-754 binary32。
/// 刻意不用 memcpy 整个结构体、也不依赖主机字节序，避免对齐、填充和 ABI 差异
/// 让两端的字节布局对不上。
///
/// 所有函数都不做边界检查，调用方负责保证缓冲足够——上层的长度校验已经在
/// frame 和 codec 层完成，这里再查一遍只会拖慢 50 Hz 路径。

#ifndef UART_PROTO_DETAIL_BYTES_H_
#define UART_PROTO_DETAIL_BYTES_H_

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace uart {

inline void PutU16(uint8_t* dst, uint16_t value) {
  dst[0] = static_cast<uint8_t>(value & 0xFFu);
  dst[1] = static_cast<uint8_t>((value >> 8) & 0xFFu);
}

inline void PutU32(uint8_t* dst, uint32_t value) {
  for (size_t i = 0; i < 4; ++i) {
    dst[i] = static_cast<uint8_t>((value >> (8 * i)) & 0xFFu);
  }
}

inline void PutU64(uint8_t* dst, uint64_t value) {
  for (size_t i = 0; i < 8; ++i) {
    dst[i] = static_cast<uint8_t>((value >> (8 * i)) & 0xFFu);
  }
}

inline void PutI64(uint8_t* dst, int64_t value) { PutU64(dst, static_cast<uint64_t>(value)); }

/// 按 IEEE-754 binary32 写入。通过 memcpy 转位模式，避免 union 或指针别名的未定义行为。
inline void PutF32(uint8_t* dst, float value) {
  uint32_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  PutU32(dst, bits);
}

inline uint16_t GetU16(const uint8_t* src) {
  return static_cast<uint16_t>(src[0] | (static_cast<uint16_t>(src[1]) << 8));
}

inline uint32_t GetU32(const uint8_t* src) {
  uint32_t value = 0;
  for (size_t i = 0; i < 4; ++i) {
    value |= static_cast<uint32_t>(src[i]) << (8 * i);
  }
  return value;
}

inline uint64_t GetU64(const uint8_t* src) {
  uint64_t value = 0;
  for (size_t i = 0; i < 8; ++i) {
    value |= static_cast<uint64_t>(src[i]) << (8 * i);
  }
  return value;
}

inline int64_t GetI64(const uint8_t* src) { return static_cast<int64_t>(GetU64(src)); }

inline float GetF32(const uint8_t* src) {
  const uint32_t bits = GetU32(src);
  float value = 0.0f;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

}  // namespace uart

#endif  // UART_PROTO_DETAIL_BYTES_H_
