/// @file
/// 测试用的极简断言工具。
///
/// 刻意不引入 GoogleTest：协议核心的测试是纯计算与状态机比对，用不到 fixture、
/// mock 框架或参数化，省下一个第三方依赖比省几行样板更划算。

#ifndef UART_TESTS_CHECK_H_
#define UART_TESTS_CHECK_H_

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>

namespace uart::test {

/// 失败计数。非零时测试进程返回非 0，供 CTest 判定。
inline int g_failures = 0;

inline void Report(bool ok, const char* expr, const char* file, int line) {
  if (!ok) {
    ++g_failures;
    std::fprintf(stderr, "FAIL %s:%d: %s\n", file, line, expr);
  }
}

/// 把字节序列渲染成十六进制，断言失败时便于对照协议文档里的线上字节。
inline std::string Hex(const uint8_t* data, size_t len) {
  std::string out;
  char buf[4];
  for (size_t i = 0; i < len; ++i) {
    std::snprintf(buf, sizeof(buf), "%02X", data[i]);
    if (i != 0) {
      out.push_back(' ');
    }
    out.append(buf);
  }
  return out;
}

/// 汇总结果，作为 main 的返回值。
inline int Finish(const char* suite) {
  if (g_failures == 0) {
    std::printf("PASS %s\n", suite);
    return 0;
  }
  std::fprintf(stderr, "FAIL %s: %d 处失败\n", suite, g_failures);
  return 1;
}

}  // namespace uart::test

#define CHECK(expr) ::uart::test::Report((expr), #expr, __FILE__, __LINE__)

#endif  // UART_TESTS_CHECK_H_
