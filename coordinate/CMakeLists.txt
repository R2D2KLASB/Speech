
cmake_minimum_required(VERSION 3.5)
project(Coordinates)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_PREFIX_PATH "/home/ubuntu/portaudio/install")

find_package(PortAudio REQUIRED)
find_package(CURL REQUIRED)

add_executable(coordinates publisher_member_function.cpp)
target_link_libraries(coordinates  portaudio CURL::libcurl)

install(TARGETS coordinates DESTINATION lib/${PROJECT_NAME})

