/// @file
/// 内存假传输，用于在没有硬件的情况下测试帧层与会话层。
///
/// 只在测试中使用，不进 uart_core 库。它同时充当"假下位机"的下半部分：
/// 测试往 @ref FakePort::PushRx 里塞下位机会发的字节，再从 @ref FakePort::tx
/// 里检查上位机发出去的字节。

#ifndef UART_TESTS_MOCK_FAKE_H_
#define UART_TESTS_MOCK_FAKE_H_

#include <algorithm>
#include <cstdint>
#include <deque>
#include <vector>

#include "link/clock.h"
#include "link/port.h"

namespace uart::test {

/// 手动推进的假时钟。会话层的超时、50 Hz 节拍和时间同步全靠它驱动，
/// 这样"250 ms 看门狗"这类逻辑可以在微秒内测完，不用真的等。
class FakeClock final : public Clock {
 public:
  uint64_t NowUs() const override { return now_us_; }
  void AdvanceUs(uint64_t us) { now_us_ += us; }
  void AdvanceMs(uint64_t ms) { now_us_ += ms * 1000; }

 private:
  uint64_t now_us_ = 0;
};

/// 全内存的 @ref Transport 实现。
class FakePort : public Transport {
 public:
  bool IsOpen() const override { return open_; }
  void Close() override { open_ = false; }

  int Read(uint8_t* dst, size_t cap) override {
    if (!open_) {
      return -1;
    }
    if (fail_read_) {
      return -1;
    }
    // 每次最多返回 read_chunk_ 字节，用来模拟串口把一帧切成任意分片的行为。
    const size_t n = std::min({cap, rx_.size(), read_chunk_});
    for (size_t i = 0; i < n; ++i) {
      dst[i] = rx_.front();
      rx_.pop_front();
    }
    return static_cast<int>(n);
  }

  int Write(const uint8_t* src, size_t len) override {
    if (!open_ || fail_write_) {
      return -1;
    }
    tx_.insert(tx_.end(), src, src + len);
    return static_cast<int>(len);
  }

  bool WaitReadable(int) override { return open_ && !rx_.empty(); }

  /// 注入"下位机发来的"字节。
  void PushRx(const uint8_t* data, size_t len) { rx_.insert(rx_.end(), data, data + len); }
  void PushRx(const std::vector<uint8_t>& data) { PushRx(data.data(), data.size()); }

  /// 限制单次 Read 返回的字节数，默认不限制。设为 1 可逼出逐字节收帧的问题。
  void set_read_chunk(size_t n) { read_chunk_ = n; }

  /// 模拟链路损坏。
  void set_fail_read(bool fail) { fail_read_ = fail; }
  void set_fail_write(bool fail) { fail_write_ = fail; }

  /// 上位机已发出的全部字节。
  const std::vector<uint8_t>& tx() const { return tx_; }
  void ClearTx() { tx_.clear(); }

  /// 尚未被读走的注入字节数。
  size_t pending_rx() const { return rx_.size(); }

 private:
  bool open_ = true;
  bool fail_read_ = false;
  bool fail_write_ = false;
  size_t read_chunk_ = static_cast<size_t>(-1);
  std::deque<uint8_t> rx_;
  std::vector<uint8_t> tx_;
};

}  // namespace uart::test

#endif  // UART_TESTS_MOCK_FAKE_H_
