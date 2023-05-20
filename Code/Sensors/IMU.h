#ifndef CODE_IMU_H
#define CODE_IMU_H
#include<array>
class IMU {
private:
    double yawOff, pitchOff, rollOff, xOff, yOff, zOff, yawSD, pitchSD, rollSD, xSD, ySD, zSD;
public:
    double g;
    explicit IMU(int calibrationTime);
    double getPitch();
    double getRoll();
    double getYaw();
    std::array<double, 3> getX();
    std::array<double, 3> getY();
    std::array<double, 3> getZ();
    inline double ax();
    inline double ay();
    inline double az();
    inline double north();
    inline double vPitch();
    inline double vRoll();
    inline double vYaw();
};

#endif //CODE_IMU_H
