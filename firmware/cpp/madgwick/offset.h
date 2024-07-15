#pragma once

#include "madgMath.h"

#include <pybind11/pybind11.h>
#include <pybind11/stl_bind.h>

#define TIMEOUT (5) // s

namespace py = pybind11;
using namespace pybind11::literals;

class Offset{
    private:
        int timeout;
        float timer;
        IMUMath::Vector gyroOffset;
    public:
        Offset() : timeout(TIMEOUT), timer(0), gyroOffset(VECTOR_ZERO) {};
        IMUMath::Vector update(IMUMath::Vector gyro, float dt);
};
