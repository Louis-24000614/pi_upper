/// @file
/// 传输层：字节收发的抽象接口与 POSIX termios 串口实现。
///
/// 抽象出 @ref Transport 的目的是让上面的会话逻辑能在没有硬件的情况下跑起来——
/// 测试里换成内存 fake（见 tests/mock/fake.h），需要的话也能换成 TCP。
/// 本层只搬字节，不认识帧、不认识消息。

#ifndef UART_LINK_PORT_H_
#define UART_LINK_PORT_H_

#include <cstddef>
#include <cstdint>
#include <string>

namespace uart {

/// 字节流传输的最小接口。
///
/// 约定：@ref Read 与 @ref Write 都不抛异常，用返回值报错；-1 表示链路已损坏，
/// 调用方应关闭并走重连流程，而不是重试。实现不必线程安全，按设计只在通信线程里用。
class Transport {
 public:
  virtual ~Transport() = default;

  virtual bool IsOpen() const = 0;
  virtual void Close() = 0;

  /// 读取最多 @p cap 字节。
  /// @return 实际读到的字节数；0 表示当前无数据（不是错误）；-1 表示链路错误。
  virtual int Read(uint8_t* dst, size_t cap) = 0;

  /// 写入 @p len 字节，短写会在内部补齐。
  /// @return 写出的字节数（成功时等于 @p len）；-1 表示链路错误。
  virtual int Write(const uint8_t* src, size_t len) = 0;

  /// 等到可读或超时。@p timeout_ms 为 0 表示只做一次非阻塞探测。
  /// @return true 表示有数据可读；false 表示超时或出错，用 @ref IsOpen 区分。
  virtual bool WaitReadable(int timeout_ms) = 0;
};

/// 串口打开参数。
struct SerialConfig {
  /// 设备节点，例如 `/dev/ttyS7`。香橙派侧具体是哪一路 UART 尚未确定，
  /// 因此这里走配置而不是硬编码。
  std::string device;
  /// 波特率，必须与下位机一致。支持标准波特率常量对应的取值。
  unsigned baud = 921600;
};

/// POSIX termios 串口。
///
/// 以 raw 模式打开：关闭 CR/LF 转换、回显、规范模式和软硬件流控。协议是二进制的，
/// 任何字节改写都会破坏 COBS 分帧和 CRC 校验。
///
/// 析构会自动关闭，不需要手动 @ref Close。
class SerialPort : public Transport {
 public:
  SerialPort() = default;
  ~SerialPort() override;

  SerialPort(const SerialPort&) = delete;
  SerialPort& operator=(const SerialPort&) = delete;

  /// 打开并配置串口。已打开时会先关闭旧的。
  ///
  /// 打开后会丢弃收发缓冲里的残留字节：协议明确警告串口缓存中的历史速度命令可能
  /// 在重连后被误当成新命令，清空缓冲是上位机这边的第一道防线。
  ///
  /// @return 失败时返回 false，原因见 @ref last_error。
  bool Open(const SerialConfig& config);

  /// 最近一次失败的原因描述，供日志与诊断使用。
  const std::string& last_error() const { return last_error_; }

  bool IsOpen() const override { return fd_ >= 0; }
  void Close() override;
  int Read(uint8_t* dst, size_t cap) override;
  int Write(const uint8_t* src, size_t len) override;
  bool WaitReadable(int timeout_ms) override;

 private:
  int fd_ = -1;
  std::string last_error_;
};

}  // namespace uart

#endif  // UART_LINK_PORT_H_
