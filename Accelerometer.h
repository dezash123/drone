#ifndef DRONE_ACCELEROMETER_H
#define DRONE_ACCELEROMETER_H
#include "Runnable.h"

class Accelerometer: Runnable {
public:
    int port;
    float *(position[3]);
    float *(velocity[3]);
    float *(acceleration[3]);
    Accelerometer(int port, int cache);
    void run() override;
};

#endif //DRONE_ACCELEROMETER_H