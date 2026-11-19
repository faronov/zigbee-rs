# Builds only the ESP-IDF second-stage bootloader for the NCP (see ../README.md).
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(zigbee_rs_ncp_bootloader)
