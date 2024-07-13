cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

project(inference-cpp LANGUAGES C CXX)

# C++ configurations.
set(CMAKE_CXX_STANDARD 17)

# Shared library settings
if(APPLE)
  set(CMAKE_SHARED_LIBRARY_SUFFIX ".so")
elseif(WIN32)
  message(FATAL_ERROR "Windows is not supported")
endif()

# Finds PyTorch and Python libraries.
find_package(
  Python3 3.8...3.13
  COMPONENTS Interpreter Development
  REQUIRED)
find_package(pybind11 REQUIRED)

macro(build_extension)
  set(LIBRARY_NAME ${ARGV0})

  # Gets all source files.
  file(GLOB_RECURSE SRC_FILES *.c *.cpp)

  add_library(${LIBRARY_NAME} SHARED ${SRC_FILES})

  # Sets shared library properties.
  set_target_properties(${LIBRARY_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY
                                                   ${CMAKE_CURRENT_SOURCE_DIR})
  set_target_properties(${LIBRARY_NAME} PROPERTIES PREFIX "")
  set_target_properties(${LIBRARY_NAME} PROPERTIES OUTPUT_NAME ${LIBRARY_NAME})

  # Links with directories.
  target_include_directories(
    ${LIBRARY_NAME}
    PUBLIC ${Python3_INCLUDE_DIRS}
    PUBLIC ${pybind11_INCLUDE_DIRS})
  target_link_libraries(
    ${LIBRARY_NAME}
    PUBLIC ${Python3_LIBRARIES}
    PUBLIC ${pybind11_LIBRARIES})
endmacro()

# Subprojects.
add_subdirectory(can)
add_subdirectory(imu)
add_subdirectory(madgwick)
