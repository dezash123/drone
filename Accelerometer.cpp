#include "Accelerometer.h"
#include<stdlib.h>

Accelerometer::Accelerometer(int port, int cache) {
    this -> port = port;
    for (int i = 0; i < 3; i++) {
        acceleration[i] = (float *)(malloc((sizeof(float) * cache)));
        velocity[i] = (float *)(malloc((sizeof(float) * cache)));
        position[i] = (float *)(malloc((sizeof(float) * cache)));
    }
}

void setAcceleration() {

}

void setVelocity() {

}

void Accelerometer::run() {

}