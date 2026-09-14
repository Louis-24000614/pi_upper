# 分层与 uart 相同：math 不允许依赖 PWM/CLI；反向依赖会链接失败。
#
# servo_math —— 角度到脉宽。无 IO。
add_library(servo_math math/angle.cpp)
target_include_directories(servo_math PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# servo —— sysfs PWM + 舵机门面。
add_library(servo
  pwm/sysfs.cpp
  servo.cpp
)
target_link_libraries(servo PUBLIC servo_math)
target_include_directories(servo PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# servo_app —— 读 YAML 与命令行，不链进库测试（angle/pwm）。
add_library(servo_app
  app/config.cpp
  app/cli.cpp
)
target_link_libraries(servo_app PUBLIC servo)
target_include_directories(servo_app PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(servo_cli app/main.cpp)
target_link_libraries(servo_cli PRIVATE servo_app)
target_compile_definitions(servo_cli PRIVATE
  SERVO_DEFAULT_CONFIG="${CMAKE_SOURCE_DIR}/config/servo.yaml"
)

if(BUILD_TESTS)
  add_subdirectory(tests)
endif()
