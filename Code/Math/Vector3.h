#ifndef DRONE_VECTOR3_H
#define DRONE_VECTOR3_H


class Vector3 {
public:
    float x, y, z;
    Vector3(float x, float y, float z);
    Vector3 operator+(Vector3 other) const;
    Vector3 operator-(Vector3 other) const;
    Vector3 operator*(float scalar) const;
};


#endif //DRONE_VECTOR3_H
