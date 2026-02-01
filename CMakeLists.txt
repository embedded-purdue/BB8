# ESP32 CMakeLists.txt for ESP-IDF
cmake_minimum_required(VERSION 3.16)

# Set Bluepad32 paths
set(BLUEPAD32_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/components/bluepad32)
set(BTSTACK_ROOT ${BLUEPAD32_ROOT}/external/btstack)

# Add required components BEFORE including project.cmake
# Bluepad32 components are in src/components subdirectory
set(EXTRA_COMPONENT_DIRS components ${BLUEPAD32_ROOT}/src/components)

# Include ESP-IDF build system
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Set project name
project(BB8)
