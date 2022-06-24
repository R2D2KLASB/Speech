cmake_minimum_required(VERSION 3.10)

# set the project name
project(Matrix)

include_directories(/home/ubuntu/rpi-rgb-led-matrix/lib)
include_directories(/home/ubuntu/rpi-rgb-led-matrix/include)
file(GLOB_RECURSE RGB_HEADER_FILES /home/ubuntu/rpi-rgb-led-matrix/include/*.h)
file(GLOB_RECURSE RGB_SRC_FILES /home/ubuntu/rpi-rgb-led-matrix/lib/*.c*)

add_executable(matrix main.cpp animations.hpp serialib/lib/serialib.cpp ${RGB_SRC_FILES} ${RGB_HEADER_FILES})

target_link_libraries(matrix pthread)
