#include "animations.hpp"


#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

using rgb_matrix::RGBMatrix;
using rgb_matrix::Canvas;

#define SERIAL_PORT "/dev/ttyACM0"
#define BAUD_RATE 9600
#define PORT 8080

int main() {
        int sock = 0, client_fd;
        struct sockaddr_in serv_addr;
        char buffer[1024] = { '\0' };
	char* response = "ok";

        if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
            printf("\n Socket creation error \n");
            return -1;
        }
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(PORT);
        if (inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr) <= 0) {
            printf("\nInvalid address/ Address not supported \n");
            return -1;
        }
        if ((client_fd = connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr))) < 0) {
            printf("\nConnection Failed \n");
            return -1;
        }
        std::cout << "connected to server!\n";

	serialib serial;
        serial.openDevice(SERIAL_PORT, BAUD_RATE);

        RGBMatrix::Options my_defaults;
        my_defaults.hardware_mapping = "regular";
        my_defaults.chain_length = 1;
        my_defaults.rows = Rows;
        my_defaults.cols = Cols;
        my_defaults.show_refresh_rate = true;
        my_defaults.disable_hardware_pulsing = 1;
        my_defaults.show_refresh_rate = false;
        my_defaults.parallel = 2;
        rgb_matrix::RuntimeOptions runtime_defaults;
        runtime_defaults.gpio_slowdown = 4;

        Canvas *canvas = RGBMatrix::CreateFromOptions(my_defaults, runtime_defaults);
        Animations matrix(canvas, serial, xy{squareEnemyBeginX + 1, squareEnemyBeginY + 1}, xy{squarePlayerBeginX + 1, squarePlayerBeginY + 1});

	for(;;) {
          read(sock, buffer, 1024);
          printf("recieved command: %s\n", buffer);
	      send(sock, response, strlen(response), 0);

          if(strcmp(buffer, "boats") == 0) {
            std::vector<std::vector<int>> boats = {};

            read(sock, buffer, 1024);
	    printf("recieved data: %s\n", buffer);
            send(sock, response, strlen(response), 0);

	    for(unsigned int i = 2; i < strlen(buffer); i+=6){
	      std::vector<int> v = {};
	      v.push_back(int(buffer[i] - '0'));
	      v.push_back(int(buffer[i+2] - '0'));
	      boats.push_back(v);
	    }
	    matrix.setBoats(boats);
	  }
	/*else if(strcmp(buffer, "hit") == 0){
			xy position;
			bool enemy;
            // std::string buffer;
            read(sock, buffer, 1024);
            send(sock, responce, strlen(responce), 0);
			position.x = int(buffer[1] - '0');
			position.y = int(buffer[3] - '0');
			enemy = int(buffer[5] - '0');
			matrix.hit(position, enemy);
		}else if(strcmp(buffer, "miss")){
			xy position;
			bool enemy;
            // std::string buffer;
            read(sock, buffer, 1024);
            send(sock, responce, strlen(responce), 0);
			position.x = int(buffer[1] - '0');
			position.y = int(buffer[3] - '0');
			enemy = int(buffer[5] - '0');
			matrix.miss(position, enemy);
		}else if(strcmp(buffer, "getPos") == 0){
			matrix.handleInput();
		}
*/
	}
}
