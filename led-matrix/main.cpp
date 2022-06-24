// #include "led-matrix.h"
// #include "signal.h"
// #include <unistd.h>
#include "animations.hpp"
// #include "serialib/lib/serialib.h"
// #include <iostream>



using rgb_matrix::RGBMatrix;
using rgb_matrix::Canvas;

int main() {
    // Set some defaults
    RGBMatrix::Options my_defaults;
    my_defaults.hardware_mapping = "regular";  // or e.g. "adafruit-hat" or "adafruit-hat-pwm"
    my_defaults.chain_length = 1;
    my_defaults.rows = Rows;
    my_defaults.cols = Cols;
    my_defaults.show_refresh_rate = true;
    my_defaults.disable_hardware_pulsing = 1;
    my_defaults.parallel = 2;
    rgb_matrix::RuntimeOptions runtime_defaults;
    runtime_defaults.gpio_slowdown = 4;
    // If you drop privileges, the root user you start the program with
    // to be able to initialize the hardware will be switched to an unprivileged
    // user to minimize a potential security attack surface.
    // runtime_defaults.drop_privileges = 1;
    Canvas *canvas = RGBMatrix::CreateFromOptions(my_defaults, runtime_defaults);
    serialib serial;
    char errorOpening = serial.openDevice(SERIAL_PORT, 9600);
    std::vector<std::vector<std::vector<int>>> boats = {{{0,0},{0,1},{0,2}},{{0,4},{1,4},{2,4},{3,4}}};

    Animations matrix(canvas, serial, xy{squareBeginX + 1, squareBeginY + 1}, xy{19, 3});
    matrix.draw();
    matrix.setBoats(boats);
    matrix.setSquare({squareBeginX, squareBeginY}, squareLength, rgb{0,0,255});
    matrix.setSquare({18, 2}, 12,  rgb{0,0,255});
    usleep(1000000);
    matrix.miss(xy{16,16});
    for(;;){matrix.handleInput();}

}
