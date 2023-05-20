#ifndef DRONE_FLIGHTSYSTEM_H
#define DRONE_FLIGHTSYSTEM_H
#include "../Output/FlightMotor.h"
#include "Runnable.h"
class FlightSystem: Runnable {
public:
    FlightMotor motors[4];
    FlightSystem(FlightMotor motors[4]);
    void run() override;
};
#endif