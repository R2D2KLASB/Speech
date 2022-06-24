#ifndef ANIMATIONS_HPP
#define ANIMATIONS_HPP

#include <led-matrix.h>
#include <signal.h>
#include <unistd.h>

using rgb_matrix::RGBMatrix;
using rgb_matrix::Canvas;

struct xy{
    unsigned int x, y;
};

struct rgb{
    uint8_t r, g, b;
};

class Animations{
public:
    Animations(Canvas *canvas):
        canvas(canvas){}

    void draw(){
        canvas->Fill(0,0,255);
    }

    void drawHitOrMiss(){
        for(unsigned int i = 0; i < shipData.size(); i++){
            if(shipData[i][2] == 1){
                canvas->SetPixel(shipData[i][0], shipData[i][1], 255, 165, 0);
            }else{
                canvas->SetPixel(shipData[i][0], shipData[i][1], 255, 255, 255);
        	}
	}
    }

    void setCircle(xy midpoint, int radius, rgb color = rgb{255,255,255}){
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


    void ripple(xy midpoint, int startradius, rgb color, rgb bg){
        for(unsigned int i = 0; i < 43; i++){
            usleep(150000);
            if(i){
                setCircle(midpoint, startradius + i -1, bg);
                drawHitOrMiss();
            }
            if(i - 3){
                setCircle(midpoint, startradius + i - 4, bg);
                drawHitOrMiss();
            }
            if(i - 6){
                setCircle(midpoint, startradius + i - 7, bg);
                drawHitOrMiss();
            }
            setCircle(midpoint, startradius + i, color);
            setCircle(midpoint, startradius + i - 3, color);
            setCircle(midpoint, startradius + i - 6, color);
        }
    }

    void setLine(xy startpoint, unsigned int length, bool horizontal = true, rgb color = rgb{255,255,255}){
        for(unsigned int i = 0; i < length; i++){
            if(horizontal){
                canvas->SetPixel(startpoint.x + i, startpoint.y, color.r, color.g, color.b);
            }else{
                canvas->SetPixel(startpoint.x, startpoint.y + i, color.r, color.g, color.b);
            }
        }
    }

    void miss(xy position){
        shipData.push_back({position.x, position.y, 0});
        this->ripple(position, 0, rgb{137,209,254}, rgb{0,0,255});
    }

    void setSquare(xy startpoint, int size, bool filled = false, rgb color = rgb{255,255,255}){
        if(filled){
            for(unsigned int i = 0; i < size; i++){
                setLine(xy{startpoint.x, startpoint.y + i}, size);
            }
        }else{
            setLine(startpoint, size);
            setLine(xy{startpoint.x, startpoint.y+size-1}, size);
            setLine(startpoint, size, false);
            setLine(xy{startpoint.x+size-1, startpoint.y}, size, false);
        }
    }

private:
    Canvas *canvas;
    std::vector<std::vector<unsigned int>> shipData = {{4, 8, 1}, {5, 8, 1}, {6, 8, 0}};
};

#endif //ANIMATIONS_HPP
