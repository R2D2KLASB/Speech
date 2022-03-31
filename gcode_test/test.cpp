#include <iostream>
#include <string>
#include <vector>
using namespace std;

struct xy{
    float x;
    float y;
    std::string instruction;
};

vector<vector<xy>> gcodeVector = {{{0.142857, 0.095238, "G00"}}};
