#ifndef MYFALCON_H
#define MYFALCON_H

#include <falcon/core/FalconDevice.h>

bool initialise(libnifalcon::FalconDevice* falcon);

class myFalcon {
   private:
    double max_x, max_y, max_z;
    double min_x, min_y, min_z;
    double x, y, z;
    double lim_x_pos, lim_x_neg, lim_y_pos, lim_y_neg, lim_z_pos, lim_z_neg;
    bool button1Down, button2Down, button3Down, button4Down;
    int button1, button2, button3, button4;
    void update(libnifalcon::FalconDevice* falcon);

   public:
    myFalcon(libnifalcon::FalconDevice* falcon);
    void calibrate(libnifalcon::FalconDevice* falcon);
    void start(libnifalcon::FalconDevice* falcon);
    void get(double* x, double* y, double* z, int* button1, int* button2, int* button3, int* button4);
    void set(double lim_x_pos, double lim_x_neg, double lim_y_pos, double lim_y_neg, double lim_z_pos, double lim_z_neg);
};

#endif