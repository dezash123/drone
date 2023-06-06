#ifndef DRONE_SENSOR_H
#define DRONE_SENSOR_H
class Sensor {
public:
    virtual void calibrate(int time);
};
#endif //DRONE_SENSOR_H
