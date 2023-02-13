cmake_minimum_required(VERSION 3.8)
project(vl53l5cx)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-fPIC)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

add_subdirectory(uld)

ament_auto_add_library(
  ${PROJECT_NAME}
  SHARED
  src/vl53l5cx_node.cpp
  src/vl53l5cx.cpp
  src/ranging_helper.cpp
  src/gpio.cpp
)
target_link_libraries(
  ${PROJECT_NAME}
  uld
  gpiodcxx
  gpiod
)

ament_auto_add_executable(ranging_node src/ranging_node.cpp)
target_link_libraries(ranging_node ${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(
    APPEND
    AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_cppcheck
    ament_cmake_cpplint
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()

  # Exclude files of ULD
  file(
    GLOB_RECURSE
    _linter_excludes
    uld/*.c
    uld/*.h
  )
  ament_cppcheck(EXCLUDE ${_linter_excludes})
  ament_cpplint(EXCLUDE ${_linter_excludes})
endif()

ament_auto_package(INSTALL_TO_SHARE launch config)
