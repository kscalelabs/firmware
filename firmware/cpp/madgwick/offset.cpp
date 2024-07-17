#include "offset.h"
#include "madgMath.h"

#include <cmath>

#define CUTOFF_FREQUENCY (0.02f) // Hz

#define THRESHOLD (3.0f) // deg/s

#define filterCoeff (2.0f * (float) M_PI * CUTOFF_FREQUENCY)

IMUMath::Vector Offset::update(IMUMath::Vector gyro, float dt) {
    
    IMUMath::Vector newGyro = IMUMath::Add(gyro, IMUMath::Multiply(gyroOffset, -1.0f));
    // Reset timer if gyroscope not stationary
    if ((fabsf(newGyro.x) > THRESHOLD) || (fabsf(newGyro.y) > THRESHOLD) || (fabsf(newGyro.z) > THRESHOLD)) {
        timer = 0;
        return newGyro;
    }

    // Increment timer
    if (timer < timeout) {
        timer += dt;
        return newGyro;
    }

    // Update offset if timer has reached timeout
    gyroOffset = IMUMath::Add(gyroOffset, IMUMath::Multiply(newGyro, filterCoeff*dt));
    return newGyro;
}

PYBIND11_MODULE(offset, m) {
    py::class_<Offset>(m, "Offset")
        .def(py::init<>())
        .def("update", &Offset::update, "gyro"_a, "dt"_a);
}
