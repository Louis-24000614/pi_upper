/// @file
/// CRC-32C（Castagnoli）计算。下位机协议的帧尾校验用的就是这个多项式，
/// 与更常见的 CRC-32（0xEDB88320）**不通用**，不要互相替换。
///
/// 参数：反射多项式 0x82F63B78，初值 0xFFFFFFFF，输入输出均反射，
/// 最终异或 0xFFFFFFFF。校验串 "123456789" 的结果为 0xE3069283。
///
/// 本文件只做纯计算，不涉及帧结构；帧内的覆盖范围由 frame.h 决定。

#ifndef UART_PROTO_CRC32C_H_
#define UART_PROTO_CRC32C_H_

#include <cstddef>
#include <cstdint>

namespace uart {

/// 计算一段连续字节的 CRC-32C。
///
/// 线程安全（无内部状态）。@p data 为 nullptr 时 @p len 必须为 0。
/// 返回的是数值，写入帧尾时需按小端序序列化。
uint32_t Crc32c(const uint8_t* data, size_t len);

/// 分段计算 CRC-32C，用于数据不连续的场景（例如帧头与 payload 分开存放）。
///
/// 首段传入 @ref kCrc32cInit，末段结果需再调用 @ref Crc32cFinish 才是最终值。
/// 这样拆分是为了避免为了算一次 CRC 而把帧头和 payload 先拼进临时缓冲。
uint32_t Crc32cUpdate(uint32_t state, const uint8_t* data, size_t len);

/// 分段计算的初始状态。
constexpr uint32_t kCrc32cInit = 0xFFFFFFFFu;

/// 收尾运算：把 @ref Crc32cUpdate 的中间状态转换为最终 CRC 值。
inline uint32_t Crc32cFinish(uint32_t state) { return state ^ 0xFFFFFFFFu; }

}  // namespace uart

#endif  // UART_PROTO_CRC32C_H_
