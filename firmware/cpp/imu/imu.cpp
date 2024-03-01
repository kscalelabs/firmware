#include "imu.h"

void hello_world() {
    std::cout << "Hello, world!" << std::endl;
}

PYBIND11_MODULE(imu, m) {
    m.def("hello_world", hello_world);
}
