#include "led-matrix.h"
#include "signal.h"
#include <unistd.h>
#include "animations.hpp"

using rgb_matrix::RGBMatrix;
using rgb_matrix::Canvas;

// volatile bool interrupt_received = false;
// static void InterruptHandler(int signo) {
//     interrupt_received = true;
// }


int main() {
  // Set some defaults
  RGBMatrix::Options my_defaults;
  my_defaults.hardware_mapping = "regular";  // or e.g. "adafruit-hat" or "adafruit-hat-pwm"
  my_defaults.chain_length = 1;
  my_defaults.rows = 16;
  my_defaults.cols = 32;
  my_defaults.show_refresh_rate = true;
  my_defaults.disable_hardware_pulsing = 1;
  // my_defaults.parallel = 2;
  rgb_matrix::RuntimeOptions runtime_defaults;
  runtime_defaults.gpio_slowdown = 4;
  // If you drop privileges, the root user you start the program with
  // to be able to initialize the hardware will be switched to an unprivileged
  // user to minimize a potential security attack surface.
  // runtime_defaults.drop_privileges = 1;
  Canvas *canvas = RGBMatrix::CreateFromOptions(my_defaults, runtime_defaults);

  if (canvas == NULL) {
    PrintMatrixFlags(stderr, my_defaults, runtime_defaults);
    return 1;
  }

  Animations matrix(canvas);

  matrix.draw();
  matrix.setSquare(xy{0, 0}, 12);
  for(;;){
      matrix.miss(xy{5,5});
      matrix.miss(xy{16,8}); 
  }



  delete canvas;   // Make sure to delete it in the end.
}
