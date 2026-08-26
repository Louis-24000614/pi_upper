# 测试目录镜像源码分层。全部不依赖硬件与 Qt。
#
# proto 层的测试只链 uart_proto：如果哪天纯协议代码里悄悄用了串口或时钟，
# 这里会直接链接失败。
function(uart_add_test layer unit lib)
  add_executable(uart_${unit}_test ${layer}/${unit}_test.cpp)
  target_link_libraries(uart_${unit}_test PRIVATE ${lib})
  # tests 根目录入 include 路径，便于 #include "check.h" 与 "mock/fake.h"
  target_include_directories(uart_${unit}_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  add_test(NAME uart_${unit} COMMAND uart_${unit}_test)
endfunction()

foreach(unit crc32c cobs frame codec)
  uart_add_test(proto ${unit} uart_proto)
endforeach()

foreach(unit port sess)
  uart_add_test(link ${unit} uart_link)
endforeach()
