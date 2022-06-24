#include "led-matrix.h"
#include "signal.h"
#include <unistd.h>
#include "animations.hpp"
#include "serialib/lib/serialib.h"
#include <iostream>

using rgb_matrix::RGBMatrix;
using rgb_matrix::Canvas;

// volatile bool interrupt_received = false;
// static void InterruptHandler(int signo) {
//     interrupt_received = true;
// }
#define SERIAL_PORT "/dev/ttyACM0"
#define Rows 16
#define Cols 32
#define squareBeginX 2
#define squareBeginY 2
#define squareLength 12

std::vector<int> getFieldBeginCordinate(){
 	return {squareBeginX+1, squareBeginY+1};
}


int main() {
  // Set some defaults
  RGBMatrix::Options my_defaults;
  my_defaults.hardware_mapping = "regular";  // or e.g. "adafruit-hat" or "adafruit-hat-pwm"
  my_defaults.chain_length = 1;
  my_defaults.rows = Rows;
  my_defaults.cols = Cols;
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
  xy coordinates{squareBeginX + 1, squareBeginY + 1};
  serialib serial;
  char errorOpening = serial.openDevice(SERIAL_PORT, 9600);
  //if(errorOpening != 1){
  //	#define SERIAL_PORT "/dev/ttyACMO" 
  //}

  if (canvas == NULL) {
    PrintMatrixFlags(stderr, my_defaults, runtime_defaults);
    return 1;
  }

  Animations matrix(canvas);
  matrix.draw();
  matrix.setSquare({squareBeginX, squareBeginY}, squareLength);
  matrix.setSquare({18, 2}, 12);
  matrix.drawHitOrMiss();
  char input = -1;
  bool sw = false;
  for(;;){
	int error = serial.readChar(&input, 1);
	switch(input){
		case '1':
			if(coordinates
	.y - 1 <= squareBeginY | sw == true )break;
			canvas->SetPixel(coordinates
	.x, coordinates
	.y, 0,0,255);
			matrix.drawHitOrMiss();
			coordinates
	.y -= 1;
			canvas->SetPixel(coordinates
	.x, coordinates
	.y, 255,0,0);
			sw = true;
			break;

		case '4':
			if(coordinates
	.x - 1 <= squareBeginX | sw == true)break;
			canvas->SetPixel(coordinates
	.x, coordinates
	.y, 0,0,255);
			matrix.drawHitOrMiss();
			coordinates
	.x -= 1;
			canvas->SetPixel(coordinates
	.x, coordinates
	.y, 255,0,0);
			sw = true;
			break;
		case '2':
			if(coordinates
	.y + 1 >= squareBeginY + squareLength - 1 | sw == true)break;
			canvas->SetPixel(coordinates
	.x, coordinates
	.y, 0,0,255);
			matrix.drawHitOrMiss();
			coordinates
	.y += 1;
			canvas->SetPixel(coordinates
	.x, coordinates
	.y, 255,0,0);
			sw = true;
			break;
		case '3':
			if(coordinates
	.x + 1 >= squareBeginX + squareLength - 1 | sw == true)break;
			canvas->SetPixel(coordinates
	.x, coordinates
	.y, 0,0,255);
			matrix.drawHitOrMiss();
			coordinates
	.x += 1;
			canvas->SetPixel(coordinates
	.x, coordinates
	.y, 255,0,0);
			sw = true;
			break;

		case '0':
			///TODO fire functie
			sw = true;
			break;


		case '8':
			sw = false;
			break;
		}
	  }
}


