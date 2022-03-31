#include <iostream>
#include <string>
#include <vector>

struct xy{
    float x;
    float y;
    std::string instruction;
};


std::vector<std::vector<xy>> gcodeVector = {{{0.142857, 0.095238, "G00"},
                                             {0, 0.047619, "G01"},
                                             {0.142857, 0, "G01"},
                                             {0.285714, 0.047619, "G01"},
                                             {0.142857, 0.095238, "G01"},
                                             {0.857143, 0.952381, "G00"},
                                             {0.428572, 0.380952, "G01"},
                                             {0.714286, 0.952381, "G01"},
                                             {0.857143, 1, "G01"},
                                             {0.857143, 1, "G01"},
                                             {1, 0.952381, "G01"},
                                             {0.428572, 0.380952, "G01"}},
                                            {{5.625, 4, "G00"},
                                             {5, 5.333333, "G01"},
                                             {3, 6, "G01"},
                                             {2.5, 6, "G01"},
                                             {1.25, 5.333333, "G01"},
                                             {0.625, 4.666667, "G01"},
                                             {0, 3.333333, "G01"},
                                             {0, 2, "G01"},
                                             {0.625, 0.666667, "G01"},
                                             {1.875, 0, "G01"},
                                             {3.125, 0, "G01"},
                                             {4.375, 0.666667, "G01"},
                                             {5, 2, "G01"},
                                             {6.25, 6, "G01"},
                                             {5.625, 2.666667, "G01"},
                                             {5.625, 0.666667, "G01"},
                                             {6.25, 0, "G01"},
                                             {6.875, 0, "G01"},
                                             {8.125, 0.666667, "G01"},
                                             {8.75, 1.333333, "G01"},
                                             {10, 3.333333, "G01"}}
                                           };

int main(){
    std::string gcode = "G21\n"
                        "G90\n";
    for(unsigned int i = 0; i < 3; i++){
        int place = i * 10;
        for(unsigned int j = 0; j < gcodeVector[1].size(); j++){
            if(gcodeVector[1][j].instruction == "G00"){
                float x = gcodeVector[1][j].x + place;
                gcode += "M10\n"
                         "G00 X" + std::to_string(x) + " Y" +  std::to_string(gcodeVector[1][j].y) + "\n"
                         "M09\n";
            }else{
                float x = gcodeVector[1][j].x + place;
                gcode += "G01 X" + std::to_string(x) + " Y" +  std::to_string(gcodeVector[1][j].y) + "\n";
            }
        }
    }
    gcode += "M10\n"
             "G00 X0 Y0\n"
             "M02";

    std::cout << gcode;
}
