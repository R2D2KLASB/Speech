# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ubuntu/zeeslag_speech/led-matrix

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ubuntu/zeeslag_speech/led-matrix

# Include any dependencies generated for this target.
include CMakeFiles/matrix.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/matrix.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/matrix.dir/flags.make

CMakeFiles/matrix.dir/main.cpp.o: CMakeFiles/matrix.dir/flags.make
CMakeFiles/matrix.dir/main.cpp.o: main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/zeeslag_speech/led-matrix/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/matrix.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/matrix.dir/main.cpp.o -c /home/ubuntu/zeeslag_speech/led-matrix/main.cpp

CMakeFiles/matrix.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/matrix.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/zeeslag_speech/led-matrix/main.cpp > CMakeFiles/matrix.dir/main.cpp.i

CMakeFiles/matrix.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/matrix.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/zeeslag_speech/led-matrix/main.cpp -o CMakeFiles/matrix.dir/main.cpp.s

CMakeFiles/matrix.dir/serialib/lib/serialib.cpp.o: CMakeFiles/matrix.dir/flags.make
CMakeFiles/matrix.dir/serialib/lib/serialib.cpp.o: serialib/lib/serialib.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/zeeslag_speech/led-matrix/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/matrix.dir/serialib/lib/serialib.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/matrix.dir/serialib/lib/serialib.cpp.o -c /home/ubuntu/zeeslag_speech/led-matrix/serialib/lib/serialib.cpp

CMakeFiles/matrix.dir/serialib/lib/serialib.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/matrix.dir/serialib/lib/serialib.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/zeeslag_speech/led-matrix/serialib/lib/serialib.cpp > CMakeFiles/matrix.dir/serialib/lib/serialib.cpp.i

CMakeFiles/matrix.dir/serialib/lib/serialib.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/matrix.dir/serialib/lib/serialib.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/zeeslag_speech/led-matrix/serialib/lib/serialib.cpp -o CMakeFiles/matrix.dir/serialib/lib/serialib.cpp.s

CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/bdf-font.cc.o: CMakeFiles/matrix.dir/flags.make
CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/bdf-font.cc.o: /home/ubuntu/rpi-rgb-led-matrix/lib/bdf-font.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/zeeslag_speech/led-matrix/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/bdf-font.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/bdf-font.cc.o -c /home/ubuntu/rpi-rgb-led-matrix/lib/bdf-font.cc

CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/bdf-font.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/bdf-font.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/rpi-rgb-led-matrix/lib/bdf-font.cc > CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/bdf-font.cc.i

CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/bdf-font.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/bdf-font.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/rpi-rgb-led-matrix/lib/bdf-font.cc -o CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/bdf-font.cc.s

CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/content-streamer.cc.o: CMakeFiles/matrix.dir/flags.make
CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/content-streamer.cc.o: /home/ubuntu/rpi-rgb-led-matrix/lib/content-streamer.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/zeeslag_speech/led-matrix/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/content-streamer.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/content-streamer.cc.o -c /home/ubuntu/rpi-rgb-led-matrix/lib/content-streamer.cc

CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/content-streamer.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/content-streamer.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/rpi-rgb-led-matrix/lib/content-streamer.cc > CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/content-streamer.cc.i

CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/content-streamer.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/content-streamer.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/rpi-rgb-led-matrix/lib/content-streamer.cc -o CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/content-streamer.cc.s

CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/framebuffer.cc.o: CMakeFiles/matrix.dir/flags.make
CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/framebuffer.cc.o: /home/ubuntu/rpi-rgb-led-matrix/lib/framebuffer.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/zeeslag_speech/led-matrix/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/framebuffer.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/framebuffer.cc.o -c /home/ubuntu/rpi-rgb-led-matrix/lib/framebuffer.cc

CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/framebuffer.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/framebuffer.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/rpi-rgb-led-matrix/lib/framebuffer.cc > CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/framebuffer.cc.i

CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/framebuffer.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/framebuffer.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/rpi-rgb-led-matrix/lib/framebuffer.cc -o CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/framebuffer.cc.s

CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/gpio.cc.o: CMakeFiles/matrix.dir/flags.make
CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/gpio.cc.o: /home/ubuntu/rpi-rgb-led-matrix/lib/gpio.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/zeeslag_speech/led-matrix/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/gpio.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/gpio.cc.o -c /home/ubuntu/rpi-rgb-led-matrix/lib/gpio.cc

CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/gpio.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/gpio.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/rpi-rgb-led-matrix/lib/gpio.cc > CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/gpio.cc.i

CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/gpio.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/gpio.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/rpi-rgb-led-matrix/lib/gpio.cc -o CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/gpio.cc.s

CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/graphics.cc.o: CMakeFiles/matrix.dir/flags.make
CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/graphics.cc.o: /home/ubuntu/rpi-rgb-led-matrix/lib/graphics.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/zeeslag_speech/led-matrix/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/graphics.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/graphics.cc.o -c /home/ubuntu/rpi-rgb-led-matrix/lib/graphics.cc

CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/graphics.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/graphics.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/rpi-rgb-led-matrix/lib/graphics.cc > CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/graphics.cc.i

CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/graphics.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/graphics.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/rpi-rgb-led-matrix/lib/graphics.cc -o CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/graphics.cc.s

CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/hardware-mapping.c.o: CMakeFiles/matrix.dir/flags.make
CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/hardware-mapping.c.o: /home/ubuntu/rpi-rgb-led-matrix/lib/hardware-mapping.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/zeeslag_speech/led-matrix/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building C object CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/hardware-mapping.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/hardware-mapping.c.o   -c /home/ubuntu/rpi-rgb-led-matrix/lib/hardware-mapping.c

CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/hardware-mapping.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/hardware-mapping.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/ubuntu/rpi-rgb-led-matrix/lib/hardware-mapping.c > CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/hardware-mapping.c.i

CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/hardware-mapping.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/hardware-mapping.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/ubuntu/rpi-rgb-led-matrix/lib/hardware-mapping.c -o CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/hardware-mapping.c.s

CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/led-matrix-c.cc.o: CMakeFiles/matrix.dir/flags.make
CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/led-matrix-c.cc.o: /home/ubuntu/rpi-rgb-led-matrix/lib/led-matrix-c.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/zeeslag_speech/led-matrix/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/led-matrix-c.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/led-matrix-c.cc.o -c /home/ubuntu/rpi-rgb-led-matrix/lib/led-matrix-c.cc

CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/led-matrix-c.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/led-matrix-c.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/rpi-rgb-led-matrix/lib/led-matrix-c.cc > CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/led-matrix-c.cc.i

CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/led-matrix-c.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/led-matrix-c.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/rpi-rgb-led-matrix/lib/led-matrix-c.cc -o CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/led-matrix-c.cc.s

CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/led-matrix.cc.o: CMakeFiles/matrix.dir/flags.make
CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/led-matrix.cc.o: /home/ubuntu/rpi-rgb-led-matrix/lib/led-matrix.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/zeeslag_speech/led-matrix/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/led-matrix.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/led-matrix.cc.o -c /home/ubuntu/rpi-rgb-led-matrix/lib/led-matrix.cc

CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/led-matrix.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/led-matrix.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/rpi-rgb-led-matrix/lib/led-matrix.cc > CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/led-matrix.cc.i

CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/led-matrix.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/led-matrix.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/rpi-rgb-led-matrix/lib/led-matrix.cc -o CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/led-matrix.cc.s

CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/multiplex-mappers.cc.o: CMakeFiles/matrix.dir/flags.make
CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/multiplex-mappers.cc.o: /home/ubuntu/rpi-rgb-led-matrix/lib/multiplex-mappers.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/zeeslag_speech/led-matrix/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building CXX object CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/multiplex-mappers.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/multiplex-mappers.cc.o -c /home/ubuntu/rpi-rgb-led-matrix/lib/multiplex-mappers.cc

CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/multiplex-mappers.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/multiplex-mappers.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/rpi-rgb-led-matrix/lib/multiplex-mappers.cc > CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/multiplex-mappers.cc.i

CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/multiplex-mappers.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/multiplex-mappers.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/rpi-rgb-led-matrix/lib/multiplex-mappers.cc -o CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/multiplex-mappers.cc.s

CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/options-initialize.cc.o: CMakeFiles/matrix.dir/flags.make
CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/options-initialize.cc.o: /home/ubuntu/rpi-rgb-led-matrix/lib/options-initialize.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/zeeslag_speech/led-matrix/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Building CXX object CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/options-initialize.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/options-initialize.cc.o -c /home/ubuntu/rpi-rgb-led-matrix/lib/options-initialize.cc

CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/options-initialize.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/options-initialize.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/rpi-rgb-led-matrix/lib/options-initialize.cc > CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/options-initialize.cc.i

CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/options-initialize.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/options-initialize.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/rpi-rgb-led-matrix/lib/options-initialize.cc -o CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/options-initialize.cc.s

CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/pixel-mapper.cc.o: CMakeFiles/matrix.dir/flags.make
CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/pixel-mapper.cc.o: /home/ubuntu/rpi-rgb-led-matrix/lib/pixel-mapper.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/zeeslag_speech/led-matrix/CMakeFiles --progress-num=$(CMAKE_PROGRESS_13) "Building CXX object CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/pixel-mapper.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/pixel-mapper.cc.o -c /home/ubuntu/rpi-rgb-led-matrix/lib/pixel-mapper.cc

CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/pixel-mapper.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/pixel-mapper.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/rpi-rgb-led-matrix/lib/pixel-mapper.cc > CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/pixel-mapper.cc.i

CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/pixel-mapper.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/pixel-mapper.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/rpi-rgb-led-matrix/lib/pixel-mapper.cc -o CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/pixel-mapper.cc.s

CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/thread.cc.o: CMakeFiles/matrix.dir/flags.make
CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/thread.cc.o: /home/ubuntu/rpi-rgb-led-matrix/lib/thread.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/zeeslag_speech/led-matrix/CMakeFiles --progress-num=$(CMAKE_PROGRESS_14) "Building CXX object CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/thread.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/thread.cc.o -c /home/ubuntu/rpi-rgb-led-matrix/lib/thread.cc

CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/thread.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/thread.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/rpi-rgb-led-matrix/lib/thread.cc > CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/thread.cc.i

CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/thread.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/thread.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/rpi-rgb-led-matrix/lib/thread.cc -o CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/thread.cc.s

# Object files for target matrix
matrix_OBJECTS = \
"CMakeFiles/matrix.dir/main.cpp.o" \
"CMakeFiles/matrix.dir/serialib/lib/serialib.cpp.o" \
"CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/bdf-font.cc.o" \
"CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/content-streamer.cc.o" \
"CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/framebuffer.cc.o" \
"CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/gpio.cc.o" \
"CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/graphics.cc.o" \
"CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/hardware-mapping.c.o" \
"CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/led-matrix-c.cc.o" \
"CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/led-matrix.cc.o" \
"CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/multiplex-mappers.cc.o" \
"CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/options-initialize.cc.o" \
"CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/pixel-mapper.cc.o" \
"CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/thread.cc.o"

# External object files for target matrix
matrix_EXTERNAL_OBJECTS =

matrix: CMakeFiles/matrix.dir/main.cpp.o
matrix: CMakeFiles/matrix.dir/serialib/lib/serialib.cpp.o
matrix: CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/bdf-font.cc.o
matrix: CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/content-streamer.cc.o
matrix: CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/framebuffer.cc.o
matrix: CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/gpio.cc.o
matrix: CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/graphics.cc.o
matrix: CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/hardware-mapping.c.o
matrix: CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/led-matrix-c.cc.o
matrix: CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/led-matrix.cc.o
matrix: CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/multiplex-mappers.cc.o
matrix: CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/options-initialize.cc.o
matrix: CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/pixel-mapper.cc.o
matrix: CMakeFiles/matrix.dir/home/ubuntu/rpi-rgb-led-matrix/lib/thread.cc.o
matrix: CMakeFiles/matrix.dir/build.make
matrix: /usr/lib/aarch64-linux-gnu/libcurl.so
matrix: CMakeFiles/matrix.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ubuntu/zeeslag_speech/led-matrix/CMakeFiles --progress-num=$(CMAKE_PROGRESS_15) "Linking CXX executable matrix"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/matrix.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/matrix.dir/build: matrix

.PHONY : CMakeFiles/matrix.dir/build

CMakeFiles/matrix.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/matrix.dir/cmake_clean.cmake
.PHONY : CMakeFiles/matrix.dir/clean

CMakeFiles/matrix.dir/depend:
	cd /home/ubuntu/zeeslag_speech/led-matrix && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu/zeeslag_speech/led-matrix /home/ubuntu/zeeslag_speech/led-matrix /home/ubuntu/zeeslag_speech/led-matrix /home/ubuntu/zeeslag_speech/led-matrix /home/ubuntu/zeeslag_speech/led-matrix/CMakeFiles/matrix.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/matrix.dir/depend

