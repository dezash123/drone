#ifndef DRONE_FLIGHTSYSTEM_H
#define DRONE_FLIGHTSYSTEM_H
#include "FlightMotor.h"
#include "../Math/Vector3.h"
#include "../Sensors/IMU.h"

class FlightSystem {
private:
    const float MAX_ANGLE;
    IMU IMU;
public:
    FlightMotor frontLeft;
    FlightMotor frontRight;
    FlightMotor backLeft;
    FlightMotor backRight;
    Vector3 desiredAngle;
    Vector3 desiredVelocity;
    FlightSystem(int calibrationTime);
    void run();
    void goTo(Vector3 vel, Vector3 angle);
};
#endif