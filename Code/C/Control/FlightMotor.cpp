#include "FlightMotor.h"

FlightMotor::FlightMotor(int port) {
    (*this).port = port;
}
void FlightMotor::setPower(float pow) {
    this->power = pow;
}