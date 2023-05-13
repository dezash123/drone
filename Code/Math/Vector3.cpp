#include "Vector3.h"

Vector3::Vector3(float x, float y, float z) {
  this -> x = x;
  this -> y = y;
  this -> z = z;
};

Vector3 operator+(Vector3 other) {
  return Vector3::Vector3(x + other.x, y + other.y, z + other.z);
};
Vector3 operator-(Vector3 other) {
  return Vector3::Vector3(x - other.x, y - other.y, z - other.z);
};
Vector3 operator*(float scalar) {
  return Vector3::Vector3(x*scalar, y*scalar, z*scalar);
};
