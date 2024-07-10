#pragma once

#include "LIS3MDL.h"
#include "LSM6DSL.h"
#include "LSM9DS0.h"
#include "LSM9DS1.h"
#include "i2c-dev.h"
#include <pybind11/pybind11.h>
#include <pybind11/stl_bind.h>
#include <stdint.h>

#define RAD_TO_DEG 57.29578
#define M_PI 3.14159265358979323846

#define GYR_GAIN 0.070

namespace py = pybind11;

using namespace pybind11::literals;

template <typename T> class vector_2d_t {
public:
  vector_2d_t() : x(0), y(0) {}
  vector_2d_t(T x, T y) : x(x), y(y) {}

  std::string toString();

  T x, y;
};

template <typename T> class vector_3d_t {
public:
  vector_3d_t() : x(0), y(0), z(0) {}
  vector_3d_t(T x, T y, T z) : x(x), y(y), z(z) {}

  std::string toString();

  T x, y, z;
};

template <typename T> class vector_4d_t {
public:
  vector_4d_t() : a(0), b(0), c(0), d(0) {}
  vector_4d_t(T a, T b, T c, T d) : a(a), b(b), c(c), d(d) {}

  std::string toString();

  T &v00() { return a; }
  T &v01() { return b; }
  T &v10() { return c; }
  T &v11() { return d; }

  T a, b, c, d;
};

template <typename T> class vector_6d_t {
public:
  vector_6d_t() : y(0), p(0), r(0), x(0), y2(0), z(0) {}
  vector_6d_t(T y, T p, T r, T x, T y2, T z)
      : y(y), p(p), r(r), x(x), y2(y2), z(z) {}

  std::string toString();

  T y, p, r, x, y2, z;
};

class angles_t {
public:
  angles_t(float yaw, float pitch, float roll)
      : yaw(yaw), pitch(pitch), roll(roll) {}

  std::string toString();

  float yaw, pitch, roll;
};

class IMU {
public:
  IMU(int bus = 1);

  vector_3d_t<int16_t> readAcc();
  vector_3d_t<int16_t> readMag();
  vector_3d_t<int16_t> readGyr();

  vector_2d_t<float> getAccAngle();
  vector_3d_t<float> getGyrRate();

  std::string versionString();
  std::string toString();

private:
  int file;
  int version;
  int bus;

  void readBlock(uint8_t command, uint8_t size, uint8_t *data);
  void selectDevice(int file, int addr);

  void writeAccReg(uint8_t reg, uint8_t value);
  void writeMagReg(uint8_t reg, uint8_t value);
  void writeGyrReg(uint8_t reg, uint8_t value);
};

class ftime_t {
public:
  ftime_t();
  ftime_t(long int sec, long int usec);

  std::string toString();
  float totalSeconds();

  ftime_t operator+(const ftime_t &other);
  ftime_t operator-(const ftime_t &other);

  long int sec;
  long int usec;
};

class KalmanFilter {
public:
  KalmanFilter(IMU &imu, float qAngle = 0.01, float qGyro = 0.0003,
               float rAngle = 0.01, float minDt = 0.02);

  angles_t step();

private:
  IMU &imu;

  float qAngle;
  float qGyro;
  float rAngle;
  float minDt;

  angles_t bias;
  angles_t kfAngle;

  ftime_t time;

  vector_4d_t<float> pitchParams;
  vector_4d_t<float> rollParams;

  void filterStep(vector_4d_t<float> &p, float accAngle, float gyrRate,
                  float &kfAngle, float &bias, float dt);
};
