#include "madgMath.h"
#include "madgwick.h"

#include <sstream>

std::string IMUMath::Euler::toString() {
  std::ostringstream ss;
  ss << "Euler<yaw=" << yaw << ", pitch=" << pitch << ", roll=" << roll << ">";
  return ss.str();
}

std::string IMUMath::Quaternion::toString() {
  std::ostringstream ss;
  ss << "Quaternion<w=" << w << ", x=" << x << ", y=" << y << ", z=" << z << ">";
  return ss.str();
}

std::string IMUMath::Vector::toString() {
  std::ostringstream ss;
  ss << "Vector<x=" << x << ", y=" << y << ", z=" << z << ">";
  return ss.str();
}


//Adapted from https://github.com/bjohnsonfl/Madgwick_Filter/blob/master/madgwickFilter.c
Madgwick::Madgwick(float beta, IMUMath::Quaternion q) : beta(beta), q(q) {}

void Madgwick::update(IMUMath::Vector gyro, IMUMath::Vector accel, IMUMath::Vector mag, float dt){
    IMUMath::Quaternion prevQ = q;

    IMUMath::Quaternion qGyroHalf = IMUMath::VectorToQuaternion(IMUMath::Multiply(gyro, 0.5f));
    IMUMath::Quaternion qDot = IMUMath::QuaternionMultiply(prevQ, qGyroHalf);

    IMUMath::Quaternion qA = IMUMath::VectorToQuaternion(accel);
    IMUMath::Quaternion qM = IMUMath::VectorToQuaternion(mag);

    qA = IMUMath::QuaternionNormalise(qA);
    qM = IMUMath::QuaternionNormalise(qM);

    IMUMath::Quaternion h = IMUMath::QuaternionMultiply(prevQ, IMUMath::QuaternionMultiply(qM, IMUMath::QuaternionConjugate(prevQ)));
    float bx = sqrt(h.y * h.y + h.z * h.z);
    float bz = h.z;

    IMUMath::Quaternion qNorm = IMUMath::QuaternionNormalise(prevQ);

    float f [6] = {}; //objective function
    float J [6][4] = {0}; //Jacobian

    //Compute objective function and Jacobian
    f[0] = 2*(qNorm.x * qNorm.z - qNorm.w * qNorm.y) - accel.x;
    f[1] = 2*(qNorm.w * qNorm.x + qNorm.y* qNorm.z) - accel.y;
    f[2] = 2*(0.5 - qNorm.x * qNorm.x - qNorm.y * qNorm.y) - accel.z;
    f[3] = 2*bx*(0.5 - qNorm.y * qNorm.y - qNorm.z * qNorm.z) + 2*bz*(qNorm.x * qNorm.z - qNorm.w * qNorm.y) - mag.x;
    f[4] = 2*bx*(qNorm.x * qNorm.y - qNorm.w * qNorm.z) + 2*bz*(qNorm.w * qNorm.x + qNorm.y * qNorm.z) - mag.y;
    f[5] = 2*bx*(qNorm.w * qNorm.y + qNorm.x * qNorm.z) + 2*bz*(0.5 - qNorm.x * qNorm.x - qNorm.y * qNorm.y) - mag.z;

    J[0][0] = -2 * qNorm.y;
    J[0][1] =  2 * qNorm.z;
    J[0][2] = -2 * qNorm.w;
    J[0][3] =  2 * qNorm.x;

    J[1][0] = 2 * qNorm.x;
    J[1][1] = 2 * qNorm.w;
    J[1][2] = 2 * qNorm.z;
    J[1][3] = 2 * qNorm.y;

    J[2][0] = 0;
    J[2][1] = -4 * qNorm.x;
    J[2][2] = -4 * qNorm.y;
    J[2][3] = 0;

    J[3][0] = -2 * bz * qNorm.y;
    J[3][1] =  2 * bz * qNorm.z;
    J[3][2] = -4 * bx * qNorm.y - 2 * bz * qNorm.w;
    J[3][3] = -4 * bx * qNorm.z + 2 * bz * qNorm.x;

    J[4][0] = -2 * bx * qNorm.z + 2 * bz * qNorm.x;
    J[4][1] =  2 * bx * qNorm.y + 2 * bz * qNorm.w;
    J[4][2] =  2 * bx * qNorm.x + 2 * bz * qNorm.z;
    J[4][3] = -2 * bx * qNorm.w + 2 * bz * qNorm.y;

    J[5][0] =  2 * bx * qNorm.y;
    J[5][1] =  2 * bx * qNorm.z - 4 * bz * qNorm.x;
    J[5][2] =  2 * bx * qNorm.w - 4 * bz * qNorm.y;
    J[5][3] =  2 * bx * qNorm.x;

    //Compute gradient
    IMUMath::Quaternion qGrad = IMUMath::Quaternion(0,0,0,0);

    qGrad.w = J[0][0] * f[0] + J[1][0] * f[1] + J[2][0] * f[2] + J[3][0] * f[3] + J[4][0] * f[4] + J[5][0] * f[5];
    qGrad.x = J[0][1] * f[0] + J[1][1] * f[1] + J[2][1] * f[2] + J[3][1] * f[3] + J[4][1] * f[4] + J[5][1] * f[5];
    qGrad.y = J[0][2] * f[0] + J[1][2] * f[1] + J[2][2] * f[2] + J[3][2] * f[3] + J[4][2] * f[4] + J[5][2] * f[5];
    qGrad.z = J[0][3] * f[0] + J[1][3] * f[1] + J[2][3] * f[2] + J[3][3] * f[3] + J[4][3] * f[4] + J[5][3] * f[5];

    qGrad = IMUMath::QuaternionNormalise(qGrad);

    //Sensor fusion
    qGrad = IMUMath::QuaternionScalarMultiply(qGrad, beta);
    qDot = IMUMath::QuaternionAdd(qDot, IMUMath::QuaternionScalarMultiply(qGrad, -1.0f));

    qDot = IMUMath::QuaternionScalarMultiply(qDot, dt);
    q = IMUMath::QuaternionAdd(prevQ, qDot);
    q = IMUMath::QuaternionNormalise(q);
    

/*
    float F_g [3] = {}; //objective function
    float J_g [3][4] = {0}; //Jacobian





    //Compute objective function and Jacobian
    F_g[0] = 2*(prevQ.x * prevQ.z - prevQ.w * prevQ.y) - qA.x;
    F_g[1] = 2*(prevQ.w * prevQ.x + prevQ.y* prevQ.z) - qA.y;
    F_g[2] = 2*(0.5 - prevQ.x * prevQ.x - prevQ.y * prevQ.y) - qA.z;

    J_g[0][0] = -2 * prevQ.y;
    J_g[0][1] =  2 * prevQ.z;
    J_g[0][2] = -2 * prevQ.w;
    J_g[0][3] =  2 * prevQ.x;
    
    J_g[1][0] = 2 * prevQ.x;
    J_g[1][1] = 2 * prevQ.w;
    J_g[1][2] = 2 * prevQ.z;
    J_g[1][3] = 2 * prevQ.y;
    
    J_g[2][0] = 0;
    J_g[2][1] = -4 * prevQ.x;
    J_g[2][2] = -4 * prevQ.y;
    J_g[2][3] = 0;

    //Compute gradient
    qGrad.w = J_g[0][0] * F_g[0] + J_g[1][0] * F_g[1] + J_g[2][0] * F_g[2];
    qGrad.x = J_g[0][1] * F_g[0] + J_g[1][1] * F_g[1] + J_g[2][1] * F_g[2];
    qGrad.y = J_g[0][2] * F_g[0] + J_g[1][2] * F_g[1] + J_g[2][2] * F_g[2];
    qGrad.z = J_g[0][3] * F_g[0] + J_g[1][3] * F_g[1] + J_g[2][3] * F_g[2];

    qGrad = IMUMath::QuaternionNormalise(qGrad);

    //Sensor fusion
    qGrad = IMUMath::QuaternionScalarMultiply(qGrad, beta);
    qDot = IMUMath::QuaternionAdd(qDot, IMUMath::QuaternionScalarMultiply(qGrad, -1.0f));
    qDot = IMUMath::QuaternionScalarMultiply(qDot, dt);
    q = IMUMath::QuaternionAdd(prevQ, qDot);
    q = IMUMath::QuaternionNormalise(q);
    */
}

IMUMath::Quaternion Madgwick::getQ(){
    return q;
}

IMUMath::Euler Madgwick::getEuler(){
    return IMUMath::QuaternionToEuler(q);
}

PYBIND11_MODULE(madgwick, m) {
    py::class_<IMUMath::Quaternion>(m, "Quaternion")
            .def(py::init<float, float, float, float>(), "w"_a = 0.0f, "x"_a = 0.0f, "y"_a = 0.0f, "z"_a = 0.0f)
            .def_readonly("w", &IMUMath::Quaternion::w)
            .def_readonly("x", &IMUMath::Quaternion::x)
            .def_readonly("y", &IMUMath::Quaternion::y)
            .def_readonly("z", &IMUMath::Quaternion::z)
            .def("__str__", &IMUMath::Quaternion::toString);

    py::class_<IMUMath::Euler>(m, "Euler")
            .def(py::init<float, float, float>(), "yaw"_a = 0.0f, "pitch"_a = 0.0f, "roll"_a = 0.0f)
            .def_readonly("yaw", &IMUMath::Euler::yaw)
            .def_readonly("pitch", &IMUMath::Euler::pitch)
            .def_readonly("roll", &IMUMath::Euler::roll)
            .def("__str__", &IMUMath::Euler::toString);

    py::class_<IMUMath::Vector>(m, "Vector")
            .def(py::init<float, float, float>(), "x"_a = 0.0f, "y"_a = 0.0f, "z"_a = 0.0f)
            .def_readonly("x", &IMUMath::Vector::x)
            .def_readonly("y", &IMUMath::Vector::y)
            .def_readonly("z", &IMUMath::Vector::z)
            .def("__str__", &IMUMath::Vector::toString);

    py::class_<Madgwick>(m, "Madgwick")
            .def(py::init<float, IMUMath::Quaternion>(), "beta"_a = 0.1f, "q"_a = IDENTITY_QUATERNION)
            .def("update", &Madgwick::update, "gyro"_a, "accel"_a, "mag"_a, "dt"_a)
            .def("getQ", &Madgwick::getQ)
            .def("getEuler", &Madgwick::getEuler);
}
