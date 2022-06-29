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
        int sock = 0, valread, client_fd;
        struct sockaddr_in serv_addr;
        char* hello = "i heard u!";
        char buffer[1024] = { 0 };

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

        // Set some defaults
        RGBMatrix::Options my_defaults;
        my_defaults.hardware_mapping = "regular";  // or e.g. "adafruit-hat" or "adafruit-hat-pwm"
        my_defaults.chain_length = 1;
        my_defaults.rows = Rows;
        my_defaults.cols = Cols;
        my_defaults.show_refresh_rate = true;
        my_defaults.disable_hardware_pulsing = 1;
        my_defaults.show_refresh_rate = false;
        my_defaults.parallel = 2;
        rgb_matrix::RuntimeOptions runtime_defaults;
        runtime_defaults.gpio_slowdown = 4;
        // If you drop privileges, the root user you start the program with
        // to be able to initialize the hardware will be switched to an unprivileged
        // user to minimize a potential security attack surface.
        // runtime_defaults.drop_privileges = 1;
        Canvas *canvas = RGBMatrix::CreateFromOptions(my_defaults, runtime_defaults);
        serialib serial;
        serial.openDevice(SERIAL_PORT, BAUD_RATE);
        // std::vector<std::vector<int> boats2 = {{0,0},{0,1},{0,2},{0,4},{1,4},{2,4},{3,4}};

	Animations matrix(canvas, serial, xy{squareEnemyBeginX + 1, squareEnemyBeginY + 1}, xy{squarePlayerBeginX + 1, squarePlayerBeginY + 1});
	//std::string command;
	for(;;){
		//std::cout << "Geef commando:\n";
		//std::cin >> command;

        valread = read(sock, buffer, 1024);
        printf("recieved: %s\n", buffer);
		send(sock, hello, strlen(hello), 0);

        if(strcmp(buffer, "boats")){
			std::vector<std::vector<int>> boats = {};
            // std::string buffer;
            valread = read(sock, buffer, 1024);
            send(sock, hello, strlen(hello), 0);
			for(unsigned int i = 2; i < strlen(buffer); i+=6){
				std::vector<int> v = {};
				v.push_back(int(buffer[i] - '0'));
				v.push_back(int(buffer[i+2] - '0'));
				boats.push_back(v);
			}
			matrix.setBoats(boats);
		}else if(strcmp(buffer, "hit")){
			xy position;
			bool enemy;
            // std::string buffer;
            valread = read(sock, buffer, 1024);
            send(sock, hello, strlen(hello), 0);
			position.x = int(buffer[1] - '0');
			position.y = int(buffer[3] - '0');
			enemy = int(buffer[5] - '0');
			matrix.hit(position, enemy);
		}else if(strcmp(buffer, "miss")){
			xy position;
			bool enemy;
            // std::string buffer;
            valread = read(sock, buffer, 1024);
            send(sock, hello, strlen(hello), 0);
			position.x = int(buffer[1] - '0');
			position.y = int(buffer[3] - '0');
			enemy = int(buffer[5] - '0');
			matrix.miss(position, enemy);
		}else if(strcmp(buffer, "getPos")){
			matrix.handleInput();
		}
	}
}
