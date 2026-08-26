/// @file
/// 帧层：18 字节固定帧头的装配与校验、CRC32C 覆盖范围、以及以 0x00 为边界的
/// 收帧状态机。
///
/// 线上一帧为 `COBS(帧头 || payload || CRC32C) || 0x00`。字段级定义见下位机仓库
/// 的 UART_PROTOCOL.md 第 3 节，本文件是它在上位机侧的实现。
///
/// 本层不认识任何消息语义，只把 message_type 和 payload 原样搬运，
/// 消息字段的解析由 codec 层负责。

#ifndef UART_PROTO_FRAME_H_
#define UART_PROTO_FRAME_H_

#include <cstddef>
#include <cstdint>
#include <functional>

#include "proto/cobs.h"

namespace uart {

/// 帧头魔数，线上字节序为 5A A5。
constexpr uint16_t kMagic = 0xA55A;

/// 当前协议主版本。
constexpr uint8_t kProtocolVersion = 1;

/// 固定帧头长度。
constexpr size_t kHeaderSize = 18;

/// 帧尾 CRC32C 长度。
constexpr size_t kCrcSize = 4;

/// COBS 解码后的最大帧长。
constexpr size_t kMaxDecodedSize = 256;

/// 最大 payload 长度，即 kMaxDecodedSize - kHeaderSize - kCrcSize。
constexpr size_t kMaxPayloadSize = kMaxDecodedSize - kHeaderSize - kCrcSize;

/// COBS 编码后连同结尾分隔符的缓冲上限。
constexpr size_t kMaxEncodedSize = CobsEncodeBound(kMaxDecodedSize) + 1;

/// 固定帧头的解析结果。
///
/// 这个结构体只在进程内使用，**不允许**直接 memcpy 上线：协议要求逐字节序列化，
/// 以避免对齐、填充和 ABI 差异导致两端布局不一致。
struct Header {
  uint8_t version = kProtocolVersion;
  uint8_t msg_type = 0;
  uint8_t flags = 0;
  uint16_t sequence = 0;
  uint16_t payload_length = 0;
  /// 发送方启动后的单调微秒时间；没有可用值时置 0。
  uint64_t timestamp_us = 0;
};

/// 解帧失败的原因。按协议要求，CRC 错误与格式错误要分别计数。
enum class FrameError {
  kOk,
  /// COBS 解码失败：编码数据内部有 0x00，或码字跨过末尾。
  kCobs,
  /// 解码长度不等于 kHeaderSize + payload_length + kCrcSize，或超出上限。
  kLength,
  /// 魔数不符。
  kMagic,
  /// reserved 字段非 0。
  kReserved,
  /// CRC32C 不匹配。
  kCrc,
};

/// 把帧头与 payload 装配成一条完整的线上帧（含 COBS 编码与结尾 0x00）。
///
/// @param header payload_length 字段由本函数按 @p payload_len 覆盖，调用方不必填。
/// @param payload 可为 nullptr，此时 @p payload_len 必须为 0。
/// @param dst 目标缓冲，建议直接给 kMaxEncodedSize 大小。
/// @return 写入 @p dst 的字节数；0 表示 payload 超长或 @p cap 不足。
size_t EncodeFrame(const Header& header, const uint8_t* payload, size_t payload_len, uint8_t* dst,
                   size_t cap);

/// 校验并解析一段**已经 COBS 解码**的原始帧。
///
/// 校验顺序为长度、魔数、reserved、CRC，与协议推荐的接收状态机一致。
/// 协议主版本**不在此校验**：版本不兼容时仍需读出帧头用于诊断，是否拒绝由会话层决定。
///
/// @param out_header 成功时填入解析结果。
/// @param out_payload 成功时指向 @p raw 内部的 payload 起始位置，生命周期随 @p raw。
FrameError DecodeRawFrame(const uint8_t* raw, size_t len, Header* out_header,
                          const uint8_t** out_payload);

/// 收帧状态机：按 0x00 边界从字节流中切出完整帧。
///
/// 不拥有线程也不做 IO，调用方把从串口读到的字节喂进来即可。单个实例非线程安全，
/// 按设计只在通信线程里使用。
class Reassembler {
 public:
  /// 解出一帧时的回调。@p payload 指向内部缓冲，回调返回后即失效，需要留存请自行拷贝。
  using FrameHandler = std::function<void(const Header&, const uint8_t* payload, size_t len)>;

  /// 链路质量统计，用于上报 /diagnostics 一类的诊断信息。
  struct Stats {
    /// 成功校验并分发的帧数。
    uint32_t frames = 0;
    /// CRC 校验失败的帧数。
    uint32_t crc_errors = 0;
    /// COBS、长度、魔数、reserved 错误的合计。
    uint32_t format_errors = 0;
    /// 因编码帧超长而丢弃的次数（通常意味着丢包后错位或对端异常）。
    uint32_t overflows = 0;
  };

  /// 喂入一段收到的字节。完整帧会在本函数内同步回调 @p handler。
  void Feed(const uint8_t* data, size_t len, const FrameHandler& handler);

  /// 丢弃当前未完成的帧，但保留统计。串口重连后应调用。
  void Reset();

  const Stats& stats() const { return stats_; }

 private:
  /// 处理一个 0x00 边界：解码、校验、分发。
  void FinishFrame(const FrameHandler& handler);

  uint8_t encoded_[kMaxEncodedSize] = {};
  size_t encoded_len_ = 0;
  /// 当前帧已超长，丢弃到下一个 0x00 为止，避免把错位数据当成新帧。
  bool discarding_ = false;
  uint8_t decoded_[kMaxDecodedSize] = {};
  Stats stats_;
};

}  // namespace uart

#endif  // UART_PROTO_FRAME_H_
