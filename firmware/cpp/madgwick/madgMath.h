#pragma once

#include <cmath>
#include <cstdint>
#include <string>

//Following adapted from https://github.com/xioTechnologies/Fusion/blob/main/Fusion/FusionMath.h
namespace IMUMath {

class Vector {
public:  
  Vector(float x, float y, float z) : x(x), y(y), z(z) {}
  std::string toString();
  float x, y, z;
};

class Quaternion {
public:
  Quaternion(float w, float x, float y, float z) : w(w), x(x), y(y), z(z) {}
  std::string toString();
  float w, x, y, z;
};

class Euler {
public:
  Euler(float yaw, float pitch, float roll) : yaw(yaw), pitch(pitch), roll(roll) {}
  std::string toString();
  float yaw, pitch, roll;
};

class Matrix {
public:
  Matrix(float xx, float xy, float xz, float yx, float yy, float yz, float zx, float zy, float zz)
      : xx(xx), xy(xy), xz(xz), yx(yx), yy(yy), yz(yz), zx(zx), zy(zy), zz(zz) {}
  //std::string toString();
  float xx, xy, xz, yx, yy, yz, zx, zy, zz;
};

#define VECTOR_ZERO IMUMath::Vector(0.0f, 0.0f, 0.0f)
#define VECTOR_ONES IMUMath::Vector(1.0f, 1.0f, 1.0f)
#define IDENTITY_QUATERNION IMUMath::Quaternion(1.0f, 0.0f, 0.0f, 0.0f)
#define IDENTITY_MATRIX IMUMath::Matrix(1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f)
#define EULER_ZERO IMUMath::Euler(0.0f, 0.0f, 0.0f)

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

static inline float DegreesToRadians(const float degrees) {
    return degrees * ((float) M_PI / 180.0f);
}

static inline float RadiansToDegrees(const float radians) {
    return radians * (180.0f / (float) M_PI);
}

static inline float Asin(const float value) {
    if (value <= -1.0f) {
        return (float) M_PI / -2.0f;
    }
    if (value >= 1.0f) {
        return (float) M_PI / 2.0f;
    }
    return std::asin(value);
}

static inline Vector Multiply(Vector v, float scalar) {
    return Vector(v.x * scalar, v.y * scalar, v.z * scalar);
}

static inline bool VectorEqual(Vector v1, Vector v2) {
    return v1.x == v2.x && v1.y == v2.y && v1.z == v2.z;
}

static inline Quaternion VectorToQuaternion(Vector v) {
    return Quaternion(0.0f, v.x, v.y, v.z);
}

static inline float FastInverseSqrt(const float x) {
    union {
        float f;
        int32_t i;
    } u = { .f = x };
    u.i = 0x5F1F1412 - (u.i >> 1);
    return u.f * (1.69000231f - 0.714158168f * x * u.f * u.f);
}

static inline Quaternion QuaternionAdd(Quaternion q1, Quaternion q2) {
    return Quaternion(q1.w + q2.w, q1.x + q2.x, q1.y + q2.y, q1.z + q2.z);
}

static inline Quaternion QuaternionMultiply(Quaternion q1, Quaternion q2) {

    return Quaternion(
        q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z,
        q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y,
        q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x,
        q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w
    );
}

static inline Quaternion QuaternionNormalise(Quaternion q) {
    const float invMag = FastInverseSqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    
    return Quaternion(q.w * invMag, q.x * invMag, q.y * invMag, q.z * invMag);
}

static inline Quaternion QuaternionScalarMultiply(Quaternion q, float scalar) {

    return Quaternion(q.w * scalar, q.x * scalar, q.y * scalar, q.z * scalar);
}

static inline Matrix QuaternionToMatrix(Quaternion q) {
    const float qwqw = q.w * q.w;
    const float qwqx = q.w * q.x;
    const float qwqy = q.w * q.y;
    const float qwqz = q.w * q.z;
    const float qxqy = q.x * q.y;
    const float qxqz = q.x * q.z;
    const float qyqz = q.y * q.z;
    const float qxqx = q.x * q.x;
    const float qyqy = q.y * q.y;
    const float qzqz = q.z * q.z;



    return Matrix(2.0f * (qwqw - 0.5f + qxqx), 2.0f * (qxqy - qwqz), 2.0f * (qxqz + qwqy),
                  2.0f * (qxqy + qwqz), 2.0f * (qwqw - 0.5f + qyqy), 2.0f * (qyqz - qwqx),
                  2.0f * (qxqz - qwqy), 2.0f * (qyqz + qwqx), 2.0f * (qwqw - 0.5f + qzqz));
}

static inline Euler QuaternionToEuler(Quaternion q) {
    float halfMinusQySquared = 0.5f - q.y * q.y; // calculate common terms to avoid repeated operations
    return Euler(
        RadiansToDegrees(std::atan2(q.w * q.z + q.x * q.y, halfMinusQySquared - q.z * q.z)),
        RadiansToDegrees(Asin(q.w * q.y - q.z * q.x)),
        RadiansToDegrees(std::atan2(q.w * q.x + q.y * q.z, halfMinusQySquared - q.x * q.x))
    );

}
} // namespace IMUMath
