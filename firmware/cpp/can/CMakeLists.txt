cmake_minimum_required(VERSION 3.12)
project(can_controller)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 11)

# Add the include directory
include_directories(${CMAKE_SOURCE_DIR})

# Find the Python interpreter and libraries
find_package(Python3 3.11 EXACT COMPONENTS Interpreter Development REQUIRED)

# Find pybind11 installed in the Anaconda environment
find_package(pybind11 CONFIG REQUIRED)

# Create a Python module from your C++ code
pybind11_add_module(can_controller can.cpp)
