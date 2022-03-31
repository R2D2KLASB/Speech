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
                                             {10, 3.333333, "G01"}},
                                            {{0, 2.857143, "G00"},
                                             {1.428571, 4.571429, "G01"},
                                             {3.571429, 7.428571, "G01"},
                                             {4.285714, 8.571428, "G01"},
                                             {5, 10.28571, "G01"},
                                             {5, 11.42857, "G01"},
                                             {4.285714, 12, "G01"},
                                             {2.857143, 11.42857, "G01"},
                                             {2.142857, 10.28571, "G01"},
                                             {1.428571, 8, "G01"},
                                             {0.714286, 4.571429, "G01"},
                                             {0, 0, "G01"},
                                             {0.714286, 1.714286, "G01"},
                                             {1.428571, 2.857143, "G01"},
                                             {2.857143, 4.571429, "G01"},
                                             {4.285714, 5.142857, "G01"},
                                             {5.714286, 5.142857, "G01"},
                                             {6.428572, 4.571429, "G01"},
                                             {6.428572, 3.428572, "G01"},
                                             {5, 2.857143, "G01"},
                                             {2.857143, 2.857143, "G01"},
                                             {4.285714, 2.285714, "G01"},
                                             {5, 0.571429, "G01"},
                                             {5.714286, 0, "G01"},
                                             {6.428572, 0, "G01"},
                                             {7.857142, 0.571429, "G01"},
                                             {8.571428, 1.142857, "G01"},
                                             {10, 2.857143, "G01"}},
                                            {{0, 3, "G01"},
                                             {1.818182, 4.8, "G01"},
                                             {2.727273, 6, "G01"},
                                             {2.727273, 4.8, "G01"},
                                             {4.545454, 3, "G01"},
                                             {5.454545, 1.8, "G01"},
                                             {5.454545, 0.6, "G01"},
                                             {3.636364, 0, "G01"},
                                             {0, 0, "G00"},
                                             {1.818182 , 0, "G01"},
                                             {5.454545, 0, "G01"},
                                             {7.272727, 0.6, "G01"},
                                             {8.18181, 1.22, "G01"},
                                             {10, 3, "G01"}}
                                           };


int main(){
    std::string word = "kaas";
    std::string gcode = "G21\n"
                        "G90\n";
    for(unsigned int i = 0; i < word.size(); i++){
        int index = 2;
        if(word[i] == 'a'){
          index = 1;
        }else if(word[i] == 's'){
          index = 3;
        }
        int place = i * 10;
        for(unsigned int j = 0; j < gcodeVector[index].size(); j++){
            if(gcodeVector[index][j].instruction == "G00"){
                float x = gcodeVector[index][j].x + place;
                gcode += "M10\n"
                         "G00 X" + std::to_string(x) + " Y" +  std::to_string(gcodeVector[index][j].y) + "\n"
                         "M09\n";
            }else{
                float x = gcodeVector[index][j].x + place;
                gcode += "G01 X" + std::to_string(x) + " Y" +  std::to_string(gcodeVector[index][j].y) + "\n";
            }
        }
    }
    gcode += "M10\n"
             "G00 X0 Y0\n"
             "M02";

    std::cout << gcode;
}
