cmake_minimum_required(VERSION 3.5)
project(zeeslag)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(/home/ubuntu/rpi-rgb-led-matrix/lib)
include_directories(/home/ubuntu/rpi-rgb-led-matrix/include)
file(GLOB_RECURSE RGB_HEADER_FILES /home/ubuntu/rpi-rgb-led-matrix/include/*.h)
file(GLOB_RECURSE RGB_SRC_FILES /home/ubuntu/rpi-rgb-led-matrix/lib/*.c*)

add_executable(listener src/listener.cpp ${RGB_SRC_FILES} ${RGB_HEADER_FILES})
ament_target_dependencies(listener rclcpp std_msgs)
install(TARGETS listener DESTINATION lib/${PROJECT_NAME})

ament_package()
