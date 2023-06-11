#ifndef CODE_IMU_H
#define CODE_IMU_H
#include<array>
#include "../Math/Vector3.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include <cstdlib>
#include <cstring>
#include <cstdint>

class IMU {
private:
    float yawOff, pitchOff, rollOff, xOff, yOff, zOff, yawSD, pitchSD, rollSD, xSD, ySD, zSD;
    void calibrate();
    24 25
public:
    float g;
    IMU(int calibrationTime);
    inline Vector3 getAngle();
    inline Vector3 getA();
    inline Vector3 getV();
    inline Vector3 getP();
    inline Vector3 getRawAngle();
    inline Vector3 getRawA();
};

#endif //CODE_IMU_H
