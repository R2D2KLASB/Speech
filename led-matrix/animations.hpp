/// @file this is the animation script for the led martix.

#ifndef ANIMATIONS_HPP
#define ANIMATIONS_HPP

#include <led-matrix.h>
#include <signal.h>
#include <unistd.h>
#include "serialib/lib/serialib.h"
#include <sstream>


#define Rows 16
#define Cols 32
#define squareEnemyBeginX 18
#define squareEnemyBeginY 16
#define squarePlayerBeginX 2
#define squarePlayerBeginY 16
#define squareLength 12

using rgb_matrix::RGBMatrix;
using rgb_matrix::Canvas;

/// @brief struct for a x, y position, int
struct xy{
    unsigned int x, y;
};

/// @brief struct for rgb values, uint8_t
struct rgb{
    uint8_t r, g, b;
};

/// @brief animation class
/// @details makes the animations on a led matix. It uses circles and lines to do so.
class Animations{
public:
    /// @brief constructor for the animations
    /// @param canvas: canvas pointer
    Animations(Canvas *canvas, serialib &serial, xy coordinatesEnemy, xy coordinatesPlayer):
            canvas(canvas),
            serial(serial),
            coordinatesEnemy(coordinatesEnemy),
            coordinatesPlayer(coordinatesPlayer)
    {
        this->draw();
        this->setSquare({squareEnemyBeginX, squareEnemyBeginY}, squareLength);
        this->setSquare({squarePlayerBeginX, squarePlayerBeginY}, squareLength);
    }

    /// @brief makes the canvas blue
    void draw(){
        canvas->Fill(0,0,255);
        // canvas->Fill(255,255,255);
    }

    ///@brief Puts the ships into boats
    /// @param ships: will be put into boats
    /// details puts ships into the boats variable, so it's stored in the class
    void setBoats(std::vector<std::vector<int>> ships){
        boats = ships;
        this->drawBoats();
    }


    /// @brief draws the boats on the canvas
    void drawBoats(){
        for(unsigned int i = 0; i < boats.size(); i++){
            canvas->SetPixel(boats[i][0] + squarePlayerBeginX + 1, boats[i][1] + squarePlayerBeginY + 1, 0, 255, 0);
        }
        for(unsigned int i = 0; i < shipData.size(); i++){
            if(shipData[i][2]){
                canvas->SetPixel(shipData[i][0], shipData[i][1], 255, 165, 0);
            } else {
                canvas->SetPixel(shipData[i][0], shipData[i][1], 255, 255, 255);
            }
        }
    }


    /// @brief sets a circle om a xy.
    /// @param midpoint uses the xy struct to place a circle
    /// @param radius the size of the circle
    /// @param color by default 255,255,255.
    void setCircle(xy midpoint, int radius, rgb color = rgb{255,255,255}){
        //Using the Mid-Point drawing algorithm.
        int x = radius, y = 0;

        canvas->SetPixel(x + midpoint.x, y + midpoint.y, color.r, color.g, color.b);

        if (radius > 0) {
            canvas->SetPixel(y + midpoint.x, x + midpoint.y, color.r, color.g, color.b);
            canvas->SetPixel(-x + midpoint.x, y + midpoint.y, color.r, color.g, color.b);
            canvas->SetPixel(midpoint.x, -x + midpoint.y, color.r, color.g, color.b);
        }

        int P = 1 - radius;
        while (x > y) {
            y++;

            if (P <= 0)
                P = P + 2 * y + 1;
            else {
                x--;
                P = P + 2 * y - 2 * x + 1;
            }
            if (x < y) {
                break;
            }
            canvas->SetPixel(x + midpoint.x, y + midpoint.y, color.r, color.g, color.b);
            canvas->SetPixel(-x + midpoint.x, y + midpoint.y, color.r, color.g, color.b);
            canvas->SetPixel(x + midpoint.x, -y + midpoint.y, color.r, color.g, color.b);
            canvas->SetPixel(-x + midpoint.x, -y + midpoint.y, color.r, color.g, color.b);


            if (x != y) {
                canvas->SetPixel(y + midpoint.x, x + midpoint.y, color.r, color.g, color.b);
                canvas->SetPixel(-y + midpoint.x, x + midpoint.y, color.r, color.g, color.b);
                canvas->SetPixel(y + midpoint.x, -x + midpoint.y, color.r, color.g, color.b);
                canvas->SetPixel(-y + midpoint.x, -x + midpoint.y, color.r, color.g, color.b);
            }
        }
    }


    ///@brief makes a ripple effect
    ///@param midpoint uses the xy struct to place a circle
    ///@param startradius radius to start the first ripple, will increase over time
    ///@param color color after the ripple, to reset the color.
    ///@param bg color of the ripple
    void ripple(xy midpoint, int startradius, rgb color, rgb bg){
        for(unsigned int i = 0; i < 43; i++){
            usleep(90000);
            if(i){
                this->setCircle(midpoint, startradius + i -1, bg);
                this->drawBoats();
                this->setSquare({squareEnemyBeginX, squareEnemyBeginY}, squareLength);
                this->setSquare({squarePlayerBeginX, squarePlayerBeginY}, squareLength);
            }
            if (i - 3) {
                setCircle(midpoint, startradius + i - 4, bg);
                this->drawBoats();
                this->setSquare({squareEnemyBeginX, squareEnemyBeginY}, squareLength);
                this->setSquare({squarePlayerBeginX, squarePlayerBeginY}, squareLength);
            }
            if (i - 6) {
                setCircle(midpoint, startradius + i - 7, bg);
                this->drawBoats();
                this->setSquare({squareEnemyBeginX, squareEnemyBeginY}, squareLength);
                this->setSquare({squarePlayerBeginX, squarePlayerBeginY}, squareLength);
            }
            this->setCircle(midpoint, startradius + i, color);
            this->setCircle(midpoint, startradius + i - 3, color);
            this->setCircle(midpoint, startradius + i - 6, color);
        }
    }


    ///@brief puts a line on the matrix
    ///@param startpoint using the xy struct sets startpoint of a line
    ///@param length the length of a line.
    ///@param horizontal default true
    ///@param color default 255,255,255
    void setLine(xy startpoint, unsigned int length, bool horizontal = true, rgb color = rgb{255,255,255}){
        for(unsigned int i = 0; i < length; i++){
            if(horizontal){
                canvas->SetPixel(startpoint.x + i, startpoint.y, color.r, color.g, color.b);
            } else {
                canvas->SetPixel(startpoint.x, startpoint.y + i, color.r, color.g, color.b);
            }
        }
    }


    ///@brief draws ripple animation.
    ///@details draws the missed animation. Also stores the it miss in the shipData
    ///@param position using xy struct starts the miss animation on the matrix
    void miss(xy position, bool enemy){
        if(enemy){
            position.x += squareEnemyBeginX + 1;
            position.y += squareEnemyBeginY + 1;
        }else{
            position.x += squarePlayerBeginX + 1;
            position.y += squarePlayerBeginY + 1;
        }
        shipData.push_back({position.x, position.y, 0});
        this->ripple(position, 0, rgb{137, 209, 254}, rgb{0, 0, 255});
    }


    ///@brief draws the hit animation.
    ///@details draws the hit explosion. Also stores the hit in the shipData
    ///@param position using xy struct starts the hit animation on the matrix
    void hit(xy position, bool enemy){
        if(enemy){
            position.x += squareEnemyBeginX + 1;
            position.y += squareEnemyBeginY + 1;
        }else{
            position.x += squarePlayerBeginX + 1;
            position.y += squarePlayerBeginY + 1;
        }
        shipData.push_back({position.x, position.y, 1});
        for(unsigned int i = 0; i <= 2; i++){
            this->setCircle(position, i);
        }
        for(unsigned int i = 1; i <= 3; i++){
            this->setCircle(position, i, rgb{255,0,0});
            usleep(99999);
            this->setCircle(position, i-1);
            usleep(99999);
            this->setCircle(position, i-1, rgb{255,165,0});
        }
        for(unsigned int i = 4; i > 0; i--){
            usleep(299999);
            this->setCircle(position, i, rgb{0,0,255});
        }
        this->setSquare({squareEnemyBeginX, squareEnemyBeginY}, squareLength);
        this->setSquare({squarePlayerBeginX, squarePlayerBeginY}, squareLength);
        this->drawBoats();
    }

    ///@brief draws a rectangle
    ///@param startpoint leftunderside of the rectangle
    ///@param size size of the rectangle
    ///@param filled default false
    ///@param color default 255,255,255
    void setSquare(xy startpoint, int size, rgb color = rgb{255,255,255}, bool filled = false){
        if(filled){
            for(unsigned int i = 0; i < size; i++){
                this->setLine(xy{startpoint.x, startpoint.y + i}, size, true, color);
            }
        } else {
            this->setLine(startpoint, size, true, color);
            this->setLine(xy{startpoint.x, startpoint.y + size - 1}, size, true, color);
            this->setLine(startpoint, size, false, color);
            this->setLine(xy{startpoint.x + size - 1, startpoint.y}, size, false, color);
        }
    }

    std::string handleInput() {
        std::ostringstream s;
        char input = -1;
        bool sw = false;
        for (;;) {
            serial.readChar(&input, 1);
            switch (input) {
                case '1':
                    if (coordinatesEnemy.y - 1 <= squareEnemyBeginY | sw) break;
                    canvas->SetPixel(coordinatesEnemy.x, coordinatesEnemy.y, 0, 0, 255);
                    this->drawBoats();
                    coordinatesEnemy.y -= 1;
                    canvas->SetPixel(coordinatesEnemy.x, coordinatesEnemy.y, 255, 0, 0);
                    sw = true;
                    break;

                case '4':
                    if (coordinatesEnemy.x - 1 <= squareEnemyBeginX | sw) break;
                    canvas->SetPixel(coordinatesEnemy.x, coordinatesEnemy.y, 0, 0, 255);
                    this->drawBoats();
                    coordinatesEnemy.x -= 1;
                    canvas->SetPixel(coordinatesEnemy.x, coordinatesEnemy.y, 255, 0, 0);
                    sw = true;
                    break;

                case '2':
                    if (coordinatesEnemy.y + 1 >= squareEnemyBeginY + squareLength - 1 | sw) break;
                    canvas->SetPixel(coordinatesEnemy.x, coordinatesEnemy.y, 0, 0, 255);
                    this->drawBoats();
                    coordinatesEnemy.y += 1;
                    canvas->SetPixel(coordinatesEnemy.x, coordinatesEnemy.y, 255, 0, 0);
                    sw = true;
                    break;

                case '3':
                    if (coordinatesEnemy.x + 1 >= squareEnemyBeginX + squareLength - 1 | sw) break;
                    canvas->SetPixel(coordinatesEnemy.x, coordinatesEnemy.y, 0, 0, 255);
                    this->drawBoats();
                    coordinatesEnemy.x += 1;
                    canvas->SetPixel(coordinatesEnemy.x, coordinatesEnemy.y, 255, 0, 0);
                    sw = true;
                    break;

                case '0':
                    s<<'('<<coordinatesEnemy.x<<','<<coordinatesEnemy.y<<')';
                    return s.str();


                case '8':
                    sw = false;
                    break;
            }
        }
    }


private:
    Canvas *canvas;
    std::vector <std::vector<unsigned int>> shipData = {};
    std::vector<std::vector<int>> boats;
    serialib &serial;
    xy coordinatesEnemy;
    xy coordinatesPlayer;
};

#endif //ANIMATIONS_HPP
