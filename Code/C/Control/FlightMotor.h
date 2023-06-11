#ifndef DRONE_FLIGHTMOTOR_H
#define DRONE_FLIGHTMOTOR_H

class FlightMotor {
public:
    int port;
    FlightMotor(int port);
    float power;
    void setPower(float pow);
};

#endif