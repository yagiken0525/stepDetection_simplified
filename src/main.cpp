#include <iostream>
#include "FootPrint.h"

using namespace std;
using namespace cv;

int main() {
    // input "PROJECT_NAME" and "VIDEO_NAME"
    FootPrint footPrint("demo1", "yagi");
    footPrint.run();
    return 0;
}