#pragma once

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl_bind.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace py = pybind11;

using namespace pybind11::literals;
