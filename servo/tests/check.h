/// @file
/// 测试用极简断言。不引入 GoogleTest，与 uart/tests/check.h 同思路。

#ifndef SERVO_TESTS_CHECK_H_
#define SERVO_TESTS_CHECK_H_

#include <cstdio>

namespace servo::test {

inline int g_failures = 0;

inline void Report(bool ok, const char* expr, const char* file, int line) {
  if (!ok) {
    ++g_failures;
    std::fprintf(stderr, "FAIL %s:%d: %s\n", file, line, expr);
  }
}

inline int Finish(const char* suite) {
  if (g_failures == 0) {
    std::printf("PASS %s\n", suite);
    return 0;
  }
  std::fprintf(stderr, "FAIL %s: %d 处失败\n", suite, g_failures);
  return 1;
}

}  // namespace servo::test

#define CHECK(expr) ::servo::test::Report((expr), #expr, __FILE__, __LINE__)

#endif  // SERVO_TESTS_CHECK_H_
