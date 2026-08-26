/// @file
/// COBS（Consistent Overhead Byte Stuffing）编解码。
///
/// 作用是把原始帧里的 0x00 全部编码掉，这样 0x00 就能当作可靠的帧边界，
/// 接收端不必依赖 UART IDLE 或读取块的边界来切帧。
///
/// 编码块的首字节是到下一个零字节（或块尾）的距离；码值 0xFF 表示后面紧跟
/// 254 个非零字节且**不隐含补零**。编码结果本身不含 0x00，结尾的分隔符
/// 0x00 由调用方（frame 层）追加，本文件不管。

#ifndef UART_PROTO_COBS_H_
#define UART_PROTO_COBS_H_

#include <cstddef>
#include <cstdint>

namespace uart {

/// 编码 @p raw_len 字节最坏情况下需要的目标缓冲大小（不含结尾分隔符）。
///
/// 每 254 个非零字节多出 1 字节码字，再加首字节码字。
constexpr size_t CobsEncodeBound(size_t raw_len) { return raw_len + raw_len / 254 + 1; }

/// 对 [@p src, @p src + @p len) 做 COBS 编码，写入 @p dst。
///
/// @param cap @p dst 的容量。不足时不写入任何内容并返回 0。
/// @return 写入的字节数；0 表示容量不足。@p len 为 0 时返回 1（单个码字 0x01）。
///
/// 输出不包含结尾的 0x00 分隔符。
size_t CobsEncode(const uint8_t* src, size_t len, uint8_t* dst, size_t cap);

/// 对一段**不含**分隔符的 COBS 数据解码，写入 @p dst。
///
/// 严格校验：编码数据内部出现 0x00、码字跨过数据末尾、或解码结果超出 @p cap
/// 都视为非法，返回 0。调用方应把返回 0 计入格式错误并丢弃整帧。
///
/// @return 解码出的字节数；0 表示输入非法或容量不足（合法输入不会解出 0 字节）。
size_t CobsDecode(const uint8_t* src, size_t len, uint8_t* dst, size_t cap);

}  // namespace uart

#endif  // UART_PROTO_COBS_H_
