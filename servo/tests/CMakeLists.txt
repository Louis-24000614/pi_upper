# 测试目录镜像源码分层。math 测试只链 servo_math。
function(servo_add_test name src lib)
  add_executable(servo_${name}_test ${src})
  target_link_libraries(servo_${name}_test PRIVATE ${lib})
  target_include_directories(servo_${name}_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  add_test(NAME servo_${name} COMMAND servo_${name}_test)
endfunction()

servo_add_test(angle math/angle_test.cpp servo_math)
servo_add_test(pwm pwm/sysfs_test.cpp servo)
servo_add_test(servo servo_test.cpp servo)
servo_add_test(app app/cli_test.cpp servo_app)
