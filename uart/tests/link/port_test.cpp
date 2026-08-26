/// @file
/// 传输层测试。
///
/// 真串口设备当前不存在（香橙派侧 UART overlay 尚未开启），所以 termios 那条路径
/// 用伪终端（PTY）来验证：PTY 走的是同一套 termios 配置，raw 模式、字节透传、
/// poll 行为都能测到，只有波特率和电平这类物理特性测不了。

#include "link/port.h"

#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>

#include <cstring>
#include <string>
#include <vector>

#include "check.h"
#include "mock/fake.h"

namespace {

using uart::SerialConfig;
using uart::SerialPort;
using uart::test::FakePort;

/// 开一对伪终端，返回主端 fd 与从端设备路径。
bool OpenPty(int* master_fd, std::string* slave_path) {
  const int fd = ::posix_openpt(O_RDWR | O_NOCTTY);
  if (fd < 0 || ::grantpt(fd) != 0 || ::unlockpt(fd) != 0) {
    return false;
  }
  const char* name = ::ptsname(fd);
  if (name == nullptr) {
    ::close(fd);
    return false;
  }
  *master_fd = fd;
  *slave_path = name;
  return true;
}

/// 二进制透传：协议帧里会出现 0x00、0x0A、0x0D，以及软件流控的 XON/XOFF
/// （0x11/0x13）。这些字节被改写或吞掉的话，COBS 分帧和 CRC 全部失效，
/// 而且现象是随机丢帧，极难定位。这条测试就是盯住这个。
void TestSerialRawPassThrough() {
  int master = -1;
  std::string slave;
  if (!OpenPty(&master, &slave)) {
    std::fprintf(stderr, "SKIP 无法创建伪终端，跳过 termios 测试\n");
    return;
  }

  SerialPort port;
  SerialConfig config;
  config.device = slave;
  config.baud = 921600;
  CHECK(port.Open(config));
  if (!port.IsOpen()) {
    std::fprintf(stderr, "  last_error: %s\n", port.last_error().c_str());
    ::close(master);
    return;
  }

  const std::vector<uint8_t> payload = {0x00, 0x0A, 0x0D, 0x11, 0x13, 0x1A, 0x5A, 0xA5, 0xFF, 0x00};

  // 下位机 -> 上位机
  CHECK(::write(master, payload.data(), payload.size()) ==
        static_cast<ssize_t>(payload.size()));
  std::vector<uint8_t> got;
  for (int spin = 0; spin < 100 && got.size() < payload.size(); ++spin) {
    if (!port.WaitReadable(50)) {
      continue;
    }
    uint8_t buf[64] = {};
    const int n = port.Read(buf, sizeof(buf));
    CHECK(n >= 0);
    if (n > 0) {
      got.insert(got.end(), buf, buf + n);
    }
  }
  CHECK(got.size() == payload.size());
  if (got.size() == payload.size()) {
    CHECK(std::memcmp(got.data(), payload.data(), payload.size()) == 0);
  }

  // 上位机 -> 下位机
  CHECK(port.Write(payload.data(), payload.size()) == static_cast<int>(payload.size()));
  std::vector<uint8_t> echoed(payload.size());
  size_t read_total = 0;
  for (int spin = 0; spin < 100 && read_total < payload.size(); ++spin) {
    const ssize_t n = ::read(master, echoed.data() + read_total, payload.size() - read_total);
    if (n > 0) {
      read_total += static_cast<size_t>(n);
    } else {
      ::usleep(1000);
    }
  }
  CHECK(read_total == payload.size());
  if (read_total == payload.size()) {
    CHECK(std::memcmp(echoed.data(), payload.data(), payload.size()) == 0);
  }

  port.Close();
  CHECK(!port.IsOpen());
  ::close(master);
}

/// 无数据时 WaitReadable 应超时返回 false，而不是阻塞或误报可读。
void TestSerialWaitTimesOut() {
  int master = -1;
  std::string slave;
  if (!OpenPty(&master, &slave)) {
    return;
  }
  SerialPort port;
  SerialConfig config;
  config.device = slave;
  CHECK(port.Open(config));
  CHECK(!port.WaitReadable(10));
  ::close(master);
}

void TestSerialOpenFailures() {
  SerialPort port;

  SerialConfig missing;
  missing.device = "/dev/tty_不存在的设备";
  CHECK(!port.Open(missing));
  CHECK(!port.IsOpen());
  CHECK(!port.last_error().empty());

  SerialConfig bad_baud;
  bad_baud.device = "/dev/null";
  bad_baud.baud = 123456;
  CHECK(!port.Open(bad_baud));
  CHECK(port.last_error().find("波特率") != std::string::npos);
}

/// 关闭后的读写必须返回错误而不是静默成功，否则会话层无法察觉链路已断。
void TestClosedPortReportsError() {
  SerialPort port;
  uint8_t buf[4] = {};
  CHECK(port.Read(buf, sizeof(buf)) == -1);
  CHECK(port.Write(buf, sizeof(buf)) == -1);
  CHECK(!port.WaitReadable(0));
}

void TestFakePortBasics() {
  FakePort fake;
  CHECK(fake.IsOpen());

  const std::vector<uint8_t> injected = {0x01, 0x00, 0x02};
  fake.PushRx(injected);
  CHECK(fake.pending_rx() == 3);
  CHECK(fake.WaitReadable(0));

  uint8_t buf[8] = {};
  CHECK(fake.Read(buf, sizeof(buf)) == 3);
  CHECK(std::memcmp(buf, injected.data(), injected.size()) == 0);
  CHECK(fake.pending_rx() == 0);
  CHECK(!fake.WaitReadable(0));
  CHECK(fake.Read(buf, sizeof(buf)) == 0);

  const uint8_t out[] = {0xAA, 0xBB};
  CHECK(fake.Write(out, sizeof(out)) == 2);
  CHECK(fake.tx().size() == 2);
  CHECK(fake.tx()[0] == 0xAA && fake.tx()[1] == 0xBB);
  fake.ClearTx();
  CHECK(fake.tx().empty());
}

/// 分片读取能力是 fake 存在的主要理由：真串口的分片边界是任意的，
/// 会话层必须对此不敏感。
void TestFakePortChunking() {
  FakePort fake;
  fake.set_read_chunk(1);
  const std::vector<uint8_t> injected = {1, 2, 3, 4};
  fake.PushRx(injected);

  uint8_t buf[8] = {};
  for (size_t i = 0; i < injected.size(); ++i) {
    CHECK(fake.Read(buf, sizeof(buf)) == 1);
    CHECK(buf[0] == injected[i]);
  }
  CHECK(fake.Read(buf, sizeof(buf)) == 0);
}

void TestFakePortFailures() {
  FakePort fake;
  uint8_t buf[4] = {};

  fake.set_fail_read(true);
  fake.PushRx(buf, sizeof(buf));
  CHECK(fake.Read(buf, sizeof(buf)) == -1);
  fake.set_fail_read(false);

  fake.set_fail_write(true);
  CHECK(fake.Write(buf, sizeof(buf)) == -1);
  fake.set_fail_write(false);

  fake.Close();
  CHECK(!fake.IsOpen());
  CHECK(fake.Read(buf, sizeof(buf)) == -1);
  CHECK(fake.Write(buf, sizeof(buf)) == -1);
}

}  // namespace

int main() {
  TestSerialRawPassThrough();
  TestSerialWaitTimesOut();
  TestSerialOpenFailures();
  TestClosedPortReportsError();
  TestFakePortBasics();
  TestFakePortChunking();
  TestFakePortFailures();
  return uart::test::Finish("port");
}
