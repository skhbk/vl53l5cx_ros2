cmake_minimum_required(VERSION 3.5)
project(vl53l5cx)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

add_subdirectory(uld)

ament_auto_add_executable(
  ranging
  src/main.cpp
  src/vl53l5cx_node.cpp
  src/vl53l5cx.cpp
  src/ranging_helper.cpp
  src/gpio.cpp
)
target_link_libraries(
  ranging
  uld
  gpiodcxx
  gpiod
)


ament_auto_package(INSTALL_TO_SHARE launch config)
