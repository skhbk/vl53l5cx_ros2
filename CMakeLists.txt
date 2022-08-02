cmake_minimum_required(VERSION 3.5)
project(vl53l5cx_pi)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_subdirectory(uld)

add_executable(ranging src/main.cpp src/vl53l5cx_node.cpp src/vl53l5cx.cpp
                       src/gpio.cpp)
target_include_directories(ranging PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(ranging uld gpiodcxx)
ament_target_dependencies(ranging rclcpp sensor_msgs std_srvs)

install(TARGETS ranging DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
ament_package()
