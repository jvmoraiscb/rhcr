#include <falcon/core/FalconDevice.h>

#include "../include/myFalcon.h"

int main(int argc, char* argv[]) {
    libnifalcon::FalconDevice falcon;

    if (!initialise(&falcon))
        return 1;

    myFalcon output = myFalcon(&falcon);

    while (true) {
        double x, y, z;
        int button1, button2, button3, button4;

        output.update(&falcon);
        output.get(&x, &y, &z, &button1, &button2, &button3, &button4);

        printf("X: %.2lf | Y:%.2lf | Z: %.2lf | b1: %d | b2: %d | b3: %d | b4: %d\n", x, y, z, button1, button2, button3, button4);
    }

    return 0;
}