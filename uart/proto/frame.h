/// @file
/// 帧层：`55 AA | TYPE | LENGTH | PAYLOAD | CRC8` 的装配与逐字节收帧状态机。
///
/// 协议 v2 相比 v1 大幅精简：不用 COBS、没有结束符、没有 18 字节固定帧头、
/// 没有序号、也没有通用时间戳。定长字段定位一帧，payload 里允许出现任意字节，
/// 包括 `55 AA` 本身——接收端严格按 LENGTH 取数据，不靠找同步字来定帧尾。
///
/// 因为没有序号，重复包与丢包检测在协议层不存在；`CMD_VEL` 靠"最后有效值覆盖"，
/// 管理命令靠 ACK 按类型配对，安全靠 ARM token、K2 确认和 250 ms 看门狗。
///
/// 字段级定义见下位机仓库的 UART_PROTOCOL.md 第 2、3 节。本层不认识消息语义，
/// 只把 TYPE 和 payload 原样搬运。

#ifndef UART_PROTO_FRAME_H_
#define UART_PROTO_FRAME_H_

#include <cstddef>
#include <cstdint>
#include <functional>

namespace uart {

/// 同步字节，线上顺序为 55 AA。
constexpr uint8_t kSync1 = 0x55;
constexpr uint8_t kSync2 = 0xAA;

/// 当前协议版本。v2 把版本号从帧头移到了 HELLO_REQ 的 payload 里。
constexpr uint8_t kProtocolVersion = 2;

/// 帧的固定开销：2 字节同步字 + TYPE + LENGTH + CRC8。
constexpr size_t kFrameOverhead = 5;

/// 固件当前限制的最大 payload 长度。
constexpr size_t kMaxPayloadSize = 128;

/// 整帧最大长度。
constexpr size_t kMaxFrameSize = kFrameOverhead + kMaxPayloadSize;

/// 字节间超时。帧收到一半突然断流时，超过这个间隔就丢掉重新找同步字。
/// 取值与固件的 `CAR_PROTOCOL_INTERBYTE_TIMEOUT_US` 保持一致。
constexpr uint64_t kInterByteTimeoutUs = 20000;

/// 装配一帧。
///
/// @param payload 可为 nullptr，此时 @p payload_len 必须为 0。
/// @param dst 目标缓冲，建议直接给 kMaxFrameSize 大小。
/// @return 写入 @p dst 的字节数；0 表示 payload 超长或 @p cap 不足。
size_t EncodeFrame(uint8_t msg_type, const uint8_t* payload, size_t payload_len, uint8_t* dst,
                   size_t cap);

/// 逐字节收帧状态机。
///
/// 不拥有线程也不做 IO，把从串口读到的字节喂进来即可。单个实例非线程安全，
/// 按设计只在通信线程里使用。
class Reassembler {
 public:
  /// 解出一帧时的回调。@p payload 指向内部缓冲，回调返回后即失效。
  using FrameHandler = std::function<void(uint8_t msg_type, const uint8_t* payload, size_t len)>;

  /// 链路质量统计，用于上报诊断信息。
  struct Stats {
    /// 成功校验并分发的帧数。
    uint32_t frames = 0;
    /// CRC8 校验失败的帧数。
    uint32_t crc_errors = 0;
    /// 字节间超时导致丢弃未完成帧的次数。
    uint32_t timeouts = 0;
    /// LENGTH 超过上限而丢弃的次数。
    uint32_t overflows = 0;
  };

  /// 喂入一段收到的字节。完整帧会在本函数内同步回调 @p handler。
  ///
  /// @param now_us 本批字节的到达时刻，用于字节间超时判断。同一批字节共用一个时刻，
  ///               这在串口按块交付的场景下足够——超时要防的是帧中途断流，不是块内抖动。
  void Feed(const uint8_t* data, size_t len, uint64_t now_us, const FrameHandler& handler);

  /// 丢弃当前未完成的帧，但保留统计。串口重连后应调用。
  void Reset();

  const Stats& stats() const { return stats_; }

 private:
  /// 与固件 `UartProtocolRxState` 一一对应的六个状态。
  enum class State {
    kWaitSync1,
    kWaitSync2,
    kReadType,
    kReadLength,
    kReadPayload,
    kReadCrc,
  };

  /// 丢弃当前帧，回到等同步字。
  void ResetFrame();

  State state_ = State::kWaitSync1;
  uint8_t msg_type_ = 0;
  uint8_t payload_len_ = 0;
  size_t payload_index_ = 0;
  uint8_t running_crc_ = 0;
  uint8_t payload_[kMaxPayloadSize] = {};
  /// 上一个字节的到达时刻，0 表示还没收到过字节。
  uint64_t last_byte_us_ = 0;
  Stats stats_;
};

}  // namespace uart

#endif  // UART_PROTO_FRAME_H_
