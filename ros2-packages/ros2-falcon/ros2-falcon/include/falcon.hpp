#ifndef FALCON_HPP
#define FALCON_HPP

#include <falcon/core/FalconDevice.h>
#include <falcon/core/FalconGeometry.h>
#include <falcon/firmware/FalconFirmwareNovintSDK.h>
#include <falcon/gmtl/gmtl.h>
#include <falcon/grip/FalconGripFourButton.h>
#include <falcon/kinematic/FalconKinematicStamper.h>
#include <falcon/kinematic/stamper/StamperUtils.h>
#include <falcon/util/FalconFirmwareBinaryNvent.h>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <memory>
#include <string>

class Falcon {
   public:
    Falcon();
    void print_info();
    void calibrate();
    void update();
    void get(double* x, double* y, double* z, int* button1, int* button2, int* button3, int* button4);
    void set(double x, double y, double z);
    void rgb(bool red, bool green, bool blue);

   private:
    libnifalcon::FalconDevice falconDevice;
    double max_x, max_y, max_z;
    double min_x, min_y, min_z;
    double x, y, z;
    double x_force, y_force, z_force;
    bool button1Down, button2Down, button3Down, button4Down;
    int button1, button2, button3, button4;

    void updatePosition(gmtl::Vec3d pos);
    void updateForces(std::array<double, 3UL> force);
};

#endif