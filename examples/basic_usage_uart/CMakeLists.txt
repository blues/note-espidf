# examples/uart_usage/CMakeLists.txt

cmake_minimum_required(VERSION 3.16)

# Set the components to include the parent directory where the notecard_esp component is
set(EXTRA_COMPONENT_DIRS ../..)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(notecard_uart_example)