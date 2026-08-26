/// @file
/// 单调时钟抽象。
///
/// 抽出接口是为了让会话层的超时、50 Hz 节拍和时间同步能在测试里用假时钟驱动——
/// 否则"250 ms 看门狗"和"命令超期发零速"这类逻辑只能靠 sleep 去碰，既慢又不稳。
///
/// 一律使用单调时钟：系统时间被 NTP 往回拨时，基于墙钟的超时判断会瞬间失效，
/// 而这条链路上的超时直接关系到刹不刹车。

#ifndef UART_LINK_CLOCK_H_
#define UART_LINK_CLOCK_H_

#include <cstdint>

namespace uart {

/// 单调时间源。
class Clock {
 public:
  virtual ~Clock() = default;

  /// 进程启动以来的单调微秒数。协议帧头的时间戳字段用的就是这个量纲。
  virtual uint64_t NowUs() const = 0;

  /// 毫秒便捷形式，超时判断用它够了。
  uint64_t NowMs() const { return NowUs() / 1000; }
};

/// 基于 `std::chrono::steady_clock` 的实现，运行时用这个。
class SteadyClock final : public Clock {
 public:
  /// 以构造时刻为零点，避免直接暴露一个巨大的绝对值。
  SteadyClock();
  uint64_t NowUs() const override;

 private:
  uint64_t origin_ns_ = 0;
};

}  // namespace uart

#endif  // UART_LINK_CLOCK_H_
