# The following four lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(BLEmesh-mqtt-on-off-client)

target_add_binary_data(BLEmesh-mqtt-on-off-client.elf "main/mqtt-eclipseprojects-io.pem" TEXT)