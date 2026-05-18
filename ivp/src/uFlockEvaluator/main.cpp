#include "FlockEvaluator.h"
#include <string>

int main(int argc, char *argv[]) {
    std::string mission_file = "meta_shoreside.moos";
    if(argc > 1) mission_file = argv[1];

    FlockEvaluator App;
    App.Run("pFlockEvaluator", mission_file.c_str());
    return 0;
}