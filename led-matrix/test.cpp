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

void ripple(Canvas *canvas, xy midpoint, int startradius, rgb color, rgb bg){
    for(unsigned int i = 0; i < 71; i++){
        usleep(500000);
        if(i){
            setCircle(canvas, midpoint, startradius + i -1, bg);
        }
        setCircle(canvas, midpoint, startradius + i, color);
    }
}

void setLine(Canvas *canvas, xy startpoint, unsigned int length, bool horizontal = true, rgb color = rgb{255,255,255}){
    for(unsigned int i = 0; i < length; i++){
        if(horizontal){
            canvas->SetPixel(startpoint.x + i, startpoint.y, color.r, color.g, color.b);
        }else{
            canvas->SetPixel(startpoint.x, startpoint.y + i, color.r, color.g, color.b);
        }
    }
}

void setSquare(Canvas *canvas, xy startpoint, int size, bool filled = false, rgb color = rgb{255,255,255}){
    if(filled){
        for(unsigned int i = 0; i < size; i++){
            setLine(canvas, xy{startpoint.x, startpoint.y + i}, size);
        }
    }else{
        setLine(canvas, startpoint, size);
        setLine(canvas, xy{startpoint.x, startpoint.y+size-1}, size);
        setLine(canvas, startpoint, size, false);
        setLine(canvas, xy{startpoint.x+size-1, startpoint.y}, size, false);
        canvas->SetPixel(5, 5, 255, 0, 0);
        canvas->SetPixel(3, 3, 255, 165, 0);
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
  my_defaults.rows = 16;
  my_defaults.cols = 32;
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
  setSquare(canvas, xy{0, 0}, 12);
  // ripple(canvas, xy{5, 5}, 0, rgb{255,0,0}, rgb{0,0,255});

  delete canvas;   // Make sure to delete it in the end.
}
