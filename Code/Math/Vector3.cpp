#include "Vector3.h"

Vector3::Vector3(float x, float y, float z) {
  this -> x = x;
  this -> y = y;
  this -> z = z;
};

Vector3 Vector3::operator+(Vector3 other) const {
  return {x + other.x, y + other.y, z + other.z};
};
Vector3 Vector3::operator-(Vector3 other) const {
  return {x - other.x, y - other.y, z - other.z};
}

Vector3 Vector3::operator*(float scalar) const {
    return {x*scalar, y*scalar, z*scalar};
};
