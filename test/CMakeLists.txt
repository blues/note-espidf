# Test CMakeLists.txt for ESP-IDF Notecard Component

cmake_minimum_required(VERSION 3.16)

# Include the component being tested
set(EXTRA_COMPONENT_DIRS "..")

# Test project configuration
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(notecard_esp_test)
