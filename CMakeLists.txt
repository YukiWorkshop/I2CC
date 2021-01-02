cmake_minimum_required(VERSION 3.14)
project(I2CC)

set(CMAKE_CXX_STANDARD 17)

include_directories(.)

add_library(I2CC INTERFACE)
target_include_directories(I2CC INTERFACE .)

add_executable(I2CC_Test test.cpp i2c-dev.h)
target_link_libraries(I2CC_Test I2CC)