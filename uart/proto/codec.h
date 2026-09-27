/// @file
/// 消息层：各消息 payload 的字段级序列化与反序列化。
///
/// 字段偏移与单位对应下位机仓库 UART_MESSAGES.md（固件 v1.2.22）。本层只做
/// 字节与结构体之间的转换，不涉及状态、时间和安全判断——那些属于 sess 层。
///
/// 长度约定：解码函数要求 payload 长度**严格相等**，不接受多余字节。
/// 非有限浮点（NaN/Inf）在两个方向上都按非法处理。

#ifndef UART_PROTO_CODEC_H_
#define UART_PROTO_CODEC_H_

#include <cstddef>
#include <cstdint>

#include "proto/msg.h"

namespace uart {

/// 各消息的 payload 长度，单位字节。DISARM 长度为 0，不需要编解码函数。
constexpr size_t kSizeHelloReq = 1;
constexpr size_t kSizeArmRequest = 4;
constexpr size_t kSizeCmdVel = 8;
constexpr size_t kSizeMotionAction = 8;
constexpr size_t kSizeAck = 2;
constexpr size_t kSizeHelloInfo = 7;
constexpr size_t kSizeOdomState = 21;
constexpr size_t kSizeImuState = 29;
constexpr size_t kSizeImuDebug = 21;
constexpr size_t kSizeSystemStatus = 7;
constexpr size_t kSizeMotionResult = 11;
constexpr size_t kSizeRfidCard = 3;

size_t EncodeHelloReq(const HelloReq& msg, uint8_t* dst, size_t cap);
size_t EncodeArmRequest(const ArmRequest& msg, uint8_t* dst, size_t cap);
size_t EncodeCmdVel(const CmdVel& msg, uint8_t* dst, size_t cap);
size_t EncodeMotionAction(const MotionAction& msg, uint8_t* dst, size_t cap);

bool DecodeHelloReq(const uint8_t* payload, size_t len, HelloReq* out);
bool DecodeArmRequest(const uint8_t* payload, size_t len, ArmRequest* out);
bool DecodeCmdVel(const uint8_t* payload, size_t len, CmdVel* out);
bool DecodeMotionAction(const uint8_t* payload, size_t len, MotionAction* out);

size_t EncodeAck(const Ack& msg, uint8_t* dst, size_t cap);
size_t EncodeHelloInfo(const HelloInfo& msg, uint8_t* dst, size_t cap);
size_t EncodeOdomState(const OdomState& msg, uint8_t* dst, size_t cap);
size_t EncodeImuState(const ImuState& msg, uint8_t* dst, size_t cap);
size_t EncodeImuDebug(const ImuDebug& msg, uint8_t* dst, size_t cap);
size_t EncodeSystemStatus(const SystemStatus& msg, uint8_t* dst, size_t cap);
size_t EncodeMotionResult(const MotionResult& msg, uint8_t* dst, size_t cap);
size_t EncodeRfidCard(const RfidCard& msg, uint8_t* dst, size_t cap);

bool DecodeAck(const uint8_t* payload, size_t len, Ack* out);
bool DecodeHelloInfo(const uint8_t* payload, size_t len, HelloInfo* out);
bool DecodeOdomState(const uint8_t* payload, size_t len, OdomState* out);
bool DecodeImuState(const uint8_t* payload, size_t len, ImuState* out);
bool DecodeImuDebug(const uint8_t* payload, size_t len, ImuDebug* out);
bool DecodeSystemStatus(const uint8_t* payload, size_t len, SystemStatus* out);
bool DecodeMotionResult(const uint8_t* payload, size_t len, MotionResult* out);
bool DecodeRfidCard(const uint8_t* payload, size_t len, RfidCard* out);

}  // namespace uart

#endif  // UART_PROTO_CODEC_H_
