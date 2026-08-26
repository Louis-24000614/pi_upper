#include "proto/cobs.h"

namespace uart {

size_t CobsEncode(const uint8_t* src, size_t len, uint8_t* dst, size_t cap) {
  if (cap < CobsEncodeBound(len)) {
    return 0;
  }

  // code_index 指向当前块预留的码字位置，块写满或遇到零字节时回填。
  size_t code_index = 0;
  size_t write = 1;
  uint8_t code = 1;

  for (size_t i = 0; i < len; ++i) {
    if (src[i] != 0) {
      dst[write++] = src[i];
      ++code;
      if (code == 0xFF) {
        // 满块：254 个非零字节，后面不隐含补零。
        dst[code_index] = code;
        code_index = write++;
        code = 1;
      }
    } else {
      dst[code_index] = code;
      code_index = write++;
      code = 1;
    }
  }
  dst[code_index] = code;
  return write;
}

size_t CobsDecode(const uint8_t* src, size_t len, uint8_t* dst, size_t cap) {
  size_t read = 0;
  size_t write = 0;

  while (read < len) {
    const uint8_t code = src[read++];
    if (code == 0) {
      return 0;  // 码字位置出现分隔符。
    }
    const size_t run = code - 1u;
    if (read + run > len || write + run > cap) {
      return 0;  // 码字跨过数据末尾，或目标缓冲不够。
    }
    for (size_t i = 0; i < run; ++i) {
      if (src[read] == 0) {
        return 0;  // 合法的 COBS 数据内部不含分隔符。
      }
      dst[write++] = src[read++];
    }
    // 非满块意味着原始数据在此处有一个零字节；数据末尾的块不补零。
    if (code != 0xFF && read < len) {
      if (write >= cap) {
        return 0;
      }
      dst[write++] = 0;
    }
  }
  return write;
}

}  // namespace uart
