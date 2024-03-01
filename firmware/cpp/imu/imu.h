#pragma once

#include "LIS3MDL.h"
#include "LSM6DSL.h"
#include "LSM9DS0.h"
#include "LSM9DS1.h"
#include "i2c-dev.h"
#include <pybind11/pybind11.h>
#include <stdint.h>

namespace py = pybind11;

using namespace pybind11::literals;

typedef struct {
  int16_t x, y, z;
} vector_3d_t;

class IMU {
public:
  IMU(int version = 99) : version(version) {
    detectIMU();
    enableIMU();
  }

  vector_3d_t readAcc();
  vector_3d_t readMag();
  vector_3d_t readGyr();

private:
  int file;
  int version;

  void readBlock(uint8_t command, uint8_t size, uint8_t *data);
  void selectDevice(int file, int addr);

  void writeAccReg(uint8_t reg, uint8_t value);
  void writeMagReg(uint8_t reg, uint8_t value);
  void writeGyrReg(uint8_t reg, uint8_t value);
  void detectIMU();
  void enableIMU();
};
