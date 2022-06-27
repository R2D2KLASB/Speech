#include "animations.hpp"

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
    serial.openDevice(SERIAL_PORT, 9600);
    std::vector<std::vector<int> boats2 = {{0,0},{0,1},{0,2},{0,4},{1,4},{2,4},{3,4}};

    Animations matrix(canvas, serial, xy{squareEnemyBeginX + 1, squareEnemyBeginY + 1}, xy{squarePlayerBeginX + 1, squarePlayerBeginY + 1});
    std::string command  = "boats";

    if(command == "boats"){
        std::vector<std::vector<int>> boats = {};
        std::string boten = "";
        std::cout  << "Geef boatposities:\n";
        std::cin >> boten;
        for(unsigned int i = 2; i < boten.size(); i+=6){
            std::vector<int> v = {};
            v.push_back(int(boten[i] - '0'));
            v.push_back(int(boten[i+2] - '0'));
            boats.push_back(v);
        }
        matrix.setBoats(boats);
    }else if(command == "hit"){
        xy position;
        std::string positie = "";
        std::cout << "Geef hitlocatie:\n";
        std::cin >> positie;
        position.x = int(positie[1] - '0');
        position.x = int(positie[3] - '0');
        enemy = int(positie[5] - '0');
        matrix.hit(position, enemy);
    }else if(command == "miss"){
        xy position;
        bool enemy;
        std::string positie = "";
        std::cout << "Geef hitlocatie:\n";
        std::cin >> positie;
        position.x = int(positie[1] - '0');
        position.y = int(positie[3] - '0');
        enemy = int(positie[5] - '0');
        matrix.miss(position, enemy);
    }else if(command == "getPos"){
        matrix.handleInput();
    }
    for(;;){
        // usleep(1000000);
        // matrix.miss(xy{2,2}, true);
        // matrix.hit(xy{5, 5}, false);
        matrix.handleInput();
    }

}
