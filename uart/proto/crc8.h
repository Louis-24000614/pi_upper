/// @file
/// CRC-8/ATM（又名 CRC-8/SMBUS）计算，协议 v2 的帧尾校验。
///
/// 参数：多项式 0x07（x⁸+x²+x+1），初值 0x00，输入输出均不反射，最终不异或。
/// 校验串 "123456789" 的结果为 0xF4。
///
/// 注意这是**不反射**的实现，与常见的反射式 CRC-8（多项式 0x8C 那类）不通用。
/// 协议 v1 用的是 CRC-32C，v2 改成了 CRC-8，两者不兼容。
///
/// 本文件只做纯计算；帧内的覆盖范围（TYPE + LENGTH + PAYLOAD，不含帧头）由 frame.h 决定。

#ifndef UART_PROTO_CRC8_H_
#define UART_PROTO_CRC8_H_

#include <cstddef>
#include <cstdint>

namespace uart {

/// 单字节推进。逐字节收帧时边收边算，避免为了校验再缓存一遍整帧。
uint8_t Crc8Update(uint8_t crc, uint8_t byte);

/// 计算一段连续字节的 CRC-8/ATM。
///
/// 线程安全（无内部状态）。@p data 为 nullptr 时 @p len 必须为 0。
uint8_t Crc8(const uint8_t* data, size_t len);

}  // namespace uart

#endif  // UART_PROTO_CRC8_H_
