# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ESP32C3-BLE-MESH)

# Tell CMake to include the Arduino component folder
list(APPEND EXTRA_COMPONENT_DIRS "components/arduino")