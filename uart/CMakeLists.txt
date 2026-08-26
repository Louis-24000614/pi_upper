# 分成两个目标，让分层在构建系统里也成立：uart_proto 不允许依赖 uart_link，
# 反向依赖写错会直接编不过，而不是靠自觉。
#
# uart_proto —— 纯协议编解码。无 IO、无状态、无时间、无 Qt，只依赖标准库。
add_library(uart_proto
  proto/crc32c.cpp
  proto/cobs.cpp
  proto/frame.cpp
  proto/codec.cpp
)

# uart_link —— 有状态、碰操作系统的部分（串口、时钟、会话状态机）。
add_library(uart_link
  link/port.cpp
  link/clock.cpp
  link/sess.cpp
)

target_link_libraries(uart_link PUBLIC uart_proto)

# include 路径统一取模块根，因此源码里的 #include 自带层级前缀
# （"proto/frame.h"、"link/sess.h"），从引用处就能看出跨了哪一层。
foreach(target uart_proto uart_link)
  target_include_directories(${target} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
endforeach()

if(BUILD_TESTS)
  add_subdirectory(tests)
endif()
