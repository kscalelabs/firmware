#pragma once

#include "madgMath.h"

#include <pybind11/pybind11.h>
#include <pybind11/stl_bind.h>

namespace py = pybind11;
using namespace pybind11::literals;

class Madgwick{
    private:
        float beta; // gain
        IMUMath::Quaternion q;

    public:
        Madgwick(float beta=0.1f, IMUMath::Quaternion q=IDENTITY_QUATERNION);
        void update(IMUMath::Vector gyro, IMUMath::Vector accel, IMUMath::Vector mag, float dt);
        IMUMath::Quaternion getQ();
        IMUMath::Euler getEuler();
};
