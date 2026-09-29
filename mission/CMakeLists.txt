add_library(mission_core
  mission_coordinator.cpp
)

target_include_directories(mission_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_compile_features(mission_core PUBLIC cxx_std_17)

target_compile_options(mission_core PRIVATE -Wall -Wextra -Wpedantic)
