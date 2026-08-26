#include "link/port.h"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace uart {
namespace {

/// 把数值波特率映射为 termios 常量。
/// 只列到工程可能用到的范围；921600 在 Linux 上是标准常量，不需要 BOTHER 特殊路径。
bool ToSpeed(unsigned baud, speed_t* out) {
  switch (baud) {
    case 9600:
      *out = B9600;
      return true;
    case 19200:
      *out = B19200;
      return true;
    case 38400:
      *out = B38400;
      return true;
    case 57600:
      *out = B57600;
      return true;
    case 115200:
      *out = B115200;
      return true;
    case 230400:
      *out = B230400;
      return true;
    case 460800:
      *out = B460800;
      return true;
    case 921600:
      *out = B921600;
      return true;
    case 1500000:
      *out = B1500000;
      return true;
    default:
      return false;
  }
}

}  // namespace

SerialPort::~SerialPort() { Close(); }

void SerialPort::Close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialPort::Open(const SerialConfig& config) {
  Close();

  speed_t speed = B921600;
  if (!ToSpeed(config.baud, &speed)) {
    last_error_ = "不支持的波特率: " + std::to_string(config.baud);
    return false;
  }

  // O_NOCTTY：不把串口当作控制终端，避免收到终端信号。
  // O_NONBLOCK：读写不阻塞，等待统一交给 poll。
  const int fd = ::open(config.device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd < 0) {
    last_error_ = "打开 " + config.device + " 失败: " + std::strerror(errno);
    return false;
  }

  termios tio{};
  if (::tcgetattr(fd, &tio) != 0) {
    last_error_ = "tcgetattr 失败: " + std::string(std::strerror(errno));
    ::close(fd);
    return false;
  }

  // raw 模式：清掉所有输入输出加工、回显、规范模式和信号生成。
  ::cfmakeraw(&tio);

  // 8N1，无硬件流控。CLOCAL 表示忽略调制解调器控制线，CREAD 打开接收。
  tio.c_cflag &= ~static_cast<tcflag_t>(PARENB | CSTOPB | CSIZE | CRTSCTS);
  tio.c_cflag |= static_cast<tcflag_t>(CS8 | CLOCAL | CREAD);
  // 软件流控必须关闭：XON/XOFF 会吃掉二进制数据里的 0x11 和 0x13。
  tio.c_iflag &= ~static_cast<tcflag_t>(IXON | IXOFF | IXANY);
  // 配合 O_NONBLOCK 与 poll，read 直接返回当前可用字节。
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;

  if (::cfsetispeed(&tio, speed) != 0 || ::cfsetospeed(&tio, speed) != 0) {
    last_error_ = "设置波特率失败: " + std::string(std::strerror(errno));
    ::close(fd);
    return false;
  }
  if (::tcsetattr(fd, TCSANOW, &tio) != 0) {
    last_error_ = "tcsetattr 失败: " + std::string(std::strerror(errno));
    ::close(fd);
    return false;
  }

  // 丢弃残留字节：重连后缓冲里的历史速度命令绝不能被当成新命令。
  ::tcflush(fd, TCIOFLUSH);

  fd_ = fd;
  last_error_.clear();
  return true;
}

int SerialPort::Read(uint8_t* dst, size_t cap) {
  if (fd_ < 0 || cap == 0) {
    return -1;
  }
  while (true) {
    const ssize_t n = ::read(fd_, dst, cap);
    if (n >= 0) {
      return static_cast<int>(n);
    }
    if (errno == EINTR) {
      continue;  // 被信号打断，重试而不是当作错误。
    }
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return 0;  // 暂时无数据。
    }
    last_error_ = "read 失败: " + std::string(std::strerror(errno));
    return -1;
  }
}

int SerialPort::Write(const uint8_t* src, size_t len) {
  if (fd_ < 0) {
    return -1;
  }
  size_t written = 0;
  while (written < len) {
    const ssize_t n = ::write(fd_, src + written, len - written);
    if (n > 0) {
      written += static_cast<size_t>(n);
      continue;
    }
    if (n < 0 && errno == EINTR) {
      continue;
    }
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      // 发送缓冲满。921600 下整帧最多 259 字节，正常不会发生；
      // 等一次可写而不是丢帧，避免把半条帧留在线上。
      pollfd pfd{fd_, POLLOUT, 0};
      if (::poll(&pfd, 1, 100) > 0) {
        continue;
      }
      last_error_ = "写超时，发送缓冲持续满";
      return -1;
    }
    last_error_ = "write 失败: " + std::string(std::strerror(errno));
    return -1;
  }
  return static_cast<int>(written);
}

bool SerialPort::WaitReadable(int timeout_ms) {
  if (fd_ < 0) {
    return false;
  }
  pollfd pfd{fd_, POLLIN, 0};
  while (true) {
    const int rc = ::poll(&pfd, 1, timeout_ms);
    if (rc > 0) {
      // 对端消失（USB 转串口被拔掉）时会报 POLLHUP/POLLERR，此时应触发重连。
      if ((pfd.revents & (POLLHUP | POLLERR | POLLNVAL)) != 0) {
        last_error_ = "串口已断开";
        Close();
        return false;
      }
      return (pfd.revents & POLLIN) != 0;
    }
    if (rc == 0) {
      return false;  // 超时
    }
    if (errno == EINTR) {
      continue;
    }
    last_error_ = "poll 失败: " + std::string(std::strerror(errno));
    return false;
  }
}

}  // namespace uart
