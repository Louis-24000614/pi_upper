/// @file
/// 消息层：各消息 payload 的字段级序列化与反序列化。
///
/// 字段偏移与单位对应下位机仓库 UART_PROTOCOL.md 第 5、6 节。本层只做字节与结构体
/// 之间的转换，不涉及状态、时间和安全判断——那些属于 sess 层。
///
/// 长度约定：解码函数要求 payload 长度**严格相等**，不接受多余字节。协议规定长度
/// 不符应回 ACK_BAD_PAYLOAD，因此这里返回 false 而不是尽力解析。
///
/// 非有限浮点（NaN/Inf）在两个方向上都按非法处理：下发时拒绝编码，接收时拒绝解码。
/// 协议明确把 NaN/Inf 归入非法 payload，让它们流进控制环或导航链路的代价太高。
///
/// MCU→主机方向的编码函数不是运行时需要的，它们供测试与仿真里的"假下位机"
/// 构造遥测帧使用，同时也让每个消息的字段布局有一条编解码往返可测。

#ifndef UART_PROTO_CODEC_H_
#define UART_PROTO_CODEC_H_

#include <cstddef>
#include <cstdint>

#include "proto/msg.h"

namespace uart {

/// 各消息的 payload 长度，单位字节。长度为 0 的消息（HELLO_REQ、DISARM、
/// RESET_ODOM、CLEAR_FAULT_REQUEST）不需要编解码函数，直接发空 payload。
constexpr size_t kSizeTimeSyncReq = 8;
constexpr size_t kSizeArmRequest = 4;
constexpr size_t kSizeCmdVel = 12;
constexpr size_t kSizeAck = 8;
constexpr size_t kSizeHelloInfo = 16;
constexpr size_t kSizeTimeSyncResp = 24;
constexpr size_t kSizeOdomState = 52;
constexpr size_t kSizeImuState = 44;
constexpr size_t kSizeImuDebug = 24;
constexpr size_t kSizeSystemStatus = 36;
constexpr size_t kSizeFaultEvent = 12;

/// 编码函数统一返回写入的字节数，0 表示缓冲不足或字段非法。
/// 解码函数统一返回是否成功，失败时 @p out 的内容未定义。

size_t EncodeTimeSyncReq(const TimeSyncReq& msg, uint8_t* dst, size_t cap);
size_t EncodeArmRequest(const ArmRequest& msg, uint8_t* dst, size_t cap);

/// 编码 CMD_VEL。线速度或角速度为 NaN/Inf 时返回 0，拒绝下发。
size_t EncodeCmdVel(const CmdVel& msg, uint8_t* dst, size_t cap);

bool DecodeTimeSyncReq(const uint8_t* payload, size_t len, TimeSyncReq* out);
bool DecodeArmRequest(const uint8_t* payload, size_t len, ArmRequest* out);
bool DecodeCmdVel(const uint8_t* payload, size_t len, CmdVel* out);

size_t EncodeAck(const Ack& msg, uint8_t* dst, size_t cap);
size_t EncodeHelloInfo(const HelloInfo& msg, uint8_t* dst, size_t cap);
size_t EncodeTimeSyncResp(const TimeSyncResp& msg, uint8_t* dst, size_t cap);
size_t EncodeOdomState(const OdomState& msg, uint8_t* dst, size_t cap);
size_t EncodeImuState(const ImuState& msg, uint8_t* dst, size_t cap);
size_t EncodeImuDebug(const ImuDebug& msg, uint8_t* dst, size_t cap);
size_t EncodeSystemStatus(const SystemStatus& msg, uint8_t* dst, size_t cap);
size_t EncodeFaultEvent(const FaultEvent& msg, uint8_t* dst, size_t cap);

bool DecodeAck(const uint8_t* payload, size_t len, Ack* out);
bool DecodeHelloInfo(const uint8_t* payload, size_t len, HelloInfo* out);
bool DecodeTimeSyncResp(const uint8_t* payload, size_t len, TimeSyncResp* out);
bool DecodeOdomState(const uint8_t* payload, size_t len, OdomState* out);
bool DecodeImuState(const uint8_t* payload, size_t len, ImuState* out);
bool DecodeImuDebug(const uint8_t* payload, size_t len, ImuDebug* out);
bool DecodeSystemStatus(const uint8_t* payload, size_t len, SystemStatus* out);
bool DecodeFaultEvent(const uint8_t* payload, size_t len, FaultEvent* out);

}  // namespace uart

#endif  // UART_PROTO_CODEC_H_
