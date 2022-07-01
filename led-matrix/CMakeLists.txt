cmake_minimum_required(VERSION 3.10)

# set the project name
project(Matrix)

include_directories(/home/ubuntu/rpi-rgb-led-matrix/lib)
include_directories(/home/ubuntu/rpi-rgb-led-matrix/include)
file(GLOB_RECURSE RGB_HEADER_FILES /home/ubuntu/rpi-rgb-led-matrix/include/*.h)
file(GLOB_RECURSE RGB_SRC_FILES /home/ubuntu/rpi-rgb-led-matrix/lib/*.c*)

list(APPEND CMAKE_PREFIX_PATH "/home/ubuntu/portaudio/install")

find_package(PortAudio REQUIRED)
find_package(CURL REQUIRED)

add_executable(matrix main.cpp animations.hpp coordinate.hpp transcription_api.hpp pa_recorder.hpp serialib/lib/serialib.cpp ${RGB_SRC_FILES} ${RGB_HEADER_FILES})

target_link_libraries(matrix portaudio CURL::libcurl pthread)
