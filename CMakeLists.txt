cmake_minimum_required(VERSION 3.16)

project(pi_upper LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(BUILD_TESTS "构建各模块 tests/ 下的测试" ON)

add_compile_options(-Wall -Wextra -Wpedantic)

if(BUILD_TESTS)
  enable_testing()
endif()

add_subdirectory(uart)
