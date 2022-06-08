#include "led-matrix.h"
#include "signal.h"
#include <unistd.h>

using rgb_matrix::RGBMatrix;
using rgb_matrix::Canvas;

struct xy{
    unsigned int x, y;
};

struct rgb{
    uint8_t r, g, b;
};

volatile bool interrupt_received = false;
static void InterruptHandler(int signo) {
  interrupt_received = true;
}

static void draw(Canvas *matrix){
    matrix->Fill(0,0,255);
    // for(;;){
    //    if(interrupt_received){
    //       return;
    //     }
    // }
}

void setCircle(Canvas *canvas, xy midpoint, int radius, rgb color = rgb{255,255,255}){
    //Using the Mid-Point drawing algorithm.
    midpoint.x += 1;
    midpoint.y += 1;
    int x = radius, y = 0;

    canvas->SetPixel(x + midpoint.x, y + midpoint.y, color.r, color.g, color.b);

    if (radius > 0){
        canvas->SetPixel(y + midpoint.x, x + midpoint.y, color.r, color.g, color.b);
        canvas->SetPixel(-x + midpoint.x, y + midpoint.y, color.r, color.g, color.b);
        canvas->SetPixel(midpoint.x, -x + midpoint.y, color.r, color.g, color.b);
    }

    int P = 1 - radius;
    while (x > y)
    {
        y++;

        if (P <= 0)
            P = P + 2*y + 1;
        else
        {
            x--;
            P = P + 2*y - 2*x + 1;
        }
        if (x < y){
            break;
        }
        canvas->SetPixel(x + midpoint.x, y + midpoint.y, color.r, color.g, color.b);
        canvas->SetPixel(-x + midpoint.x, y + midpoint.y, color.r, color.g, color.b);
        canvas->SetPixel(x + midpoint.x, -y + midpoint.y, color.r, color.g, color.b);
        canvas->SetPixel(-x + midpoint.x, -y + midpoint.y, color.r, color.g, color.b);


        if (x != y){
            canvas->SetPixel(y + midpoint.x, x + midpoint.y, color.r, color.g, color.b);
            canvas->SetPixel(-y + midpoint.x, x + midpoint.y, color.r, color.g, color.b);
            canvas->SetPixel(y + midpoint.x, -x + midpoint.y, color.r, color.g, color.b);
            canvas->SetPixel(-y + midpoint.x, -x + midpoint.y, color.r, color.g, color.b);
        }
    }
}

void ripple(Canvas *canvas, xy midpoint, int radius, rgb color, rgb bg){
    for(unsigned int i = 0; i < 71; i++){
        usleep(100000);
        if(i){
            setCircle(canvas, midpoint, radius + i -1, bg);
        }
        setCircle(canvas, midpoint, radius + i, color);
    }
    for(;;){
       if(interrupt_received){
          return;
        }
    }
}

int main() {
  // Set some defaults
  RGBMatrix::Options my_defaults;
  my_defaults.hardware_mapping = "regular";  // or e.g. "adafruit-hat" or "adafruit-hat-pwm"
  my_defaults.chain_length = 1;
  my_defaults.rows = 32;
  my_defaults.cols = 64;
  my_defaults.show_refresh_rate = true;
  my_defaults.disable_hardware_pulsing = 1;
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

  // Do your own command line handling with the remaining options.
  signal(SIGTERM, InterruptHandler);
  signal(SIGINT, InterruptHandler);

  draw(canvas);
  ripple(canvas, xy{0, 0}, 0, rgb{255,0,0}, rgb{0,0,255});

  delete canvas;   // Make sure to delete it in the end.
}
