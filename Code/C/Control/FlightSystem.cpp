#include <iostream>
#include "FlightSystem.h"
#include "FlightMotor.h"
#include "../Sensors/IMU.h"

const float MAX_ANGLE = 15;

FlightMotor frontLeft = FlightMotor(0);
FlightMotor frontRight = FlightMotor(1);
FlightMotor backLeft = FlightMotor(2);
FlightMotor backRight = FlightMotor(3);

FlightSystem::FlightSystem(int calibrationTime, IMU IMU) {
    desiredAngle = IMU.getAngle();
};