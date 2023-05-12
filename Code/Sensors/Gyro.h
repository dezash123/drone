#ifndef DRONE_GYRO_H
#define DRONE_GYRO_H
#include "Runnable.h"

class Gyro: Runnable {
public:
    int port;
    float angle;
    float angularvel;
    float angularacc;
    Gyro(int port);
    void run() override;
};

#endif //DRONE_GYRO_H