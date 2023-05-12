#ifndef DRONE_FLIGHTMOTOR_H
#define DRONE_FLIGHTMOTOR_H
#include "Runnable.h"

class FlightMotor: Runnable {
public:
    int port;
    FlightMotor(int port);
    float power;
    void setPower(float pow);
    void run() override;
};

#endif