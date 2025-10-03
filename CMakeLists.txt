# CMakeLists.txt for ESP-IDF Component Registry
cmake_minimum_required(VERSION 3.16)

# Note-c library directory
set(NOTE_C_DIR "${CMAKE_CURRENT_SOURCE_DIR}/note-c")

# Note-c source files
set(NOTE_C_SRCS
    "${NOTE_C_DIR}/n_atof.c"
    "${NOTE_C_DIR}/n_b64.c"
    "${NOTE_C_DIR}/n_cjson_helpers.c"
    "${NOTE_C_DIR}/n_cjson.c"
    "${NOTE_C_DIR}/n_cobs.c"
    "${NOTE_C_DIR}/n_const.c"
    "${NOTE_C_DIR}/n_ftoa.c"
    "${NOTE_C_DIR}/n_helpers.c"
    "${NOTE_C_DIR}/n_hooks.c"
    "${NOTE_C_DIR}/n_i2c.c"
    "${NOTE_C_DIR}/n_md5.c"
    "${NOTE_C_DIR}/n_printf.c"
    "${NOTE_C_DIR}/n_request.c"
    "${NOTE_C_DIR}/n_serial.c"
    "${NOTE_C_DIR}/n_str.c"
    "${NOTE_C_DIR}/n_ua.c"
)

# Component source files
set(COMPONENT_SRCS
    "src/notecard.c"
    "src/notecard_platform.c"
    ${NOTE_C_SRCS}
)

# Component include directories
set(COMPONENT_INCLUDES
    "include"
    "${NOTE_C_DIR}"
)

# Register the component
idf_component_register(
    SRCS ${COMPONENT_SRCS}
    INCLUDE_DIRS ${COMPONENT_INCLUDES}
    REQUIRES driver esp_timer nvs_flash
    PRIV_REQUIRES esp_system
)

# Get component version
# COMPONENT_VERSION is automatically set by IDF Component Manager when the component
# is used as a dependency. For local development, read from idf_component.yml.
if(NOT COMPONENT_VERSION)
    if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/idf_component.yml")
        file(READ "${CMAKE_CURRENT_LIST_DIR}/idf_component.yml" COMPONENT_YML)
        string(REGEX MATCH "version: \"([0-9]+\\.[0-9]+\\.[0-9]+)\"" _ "${COMPONENT_YML}")
        set(COMPONENT_VERSION ${CMAKE_MATCH_1})
    endif()
endif()

if(NOT COMPONENT_VERSION)
    set(COMPONENT_VERSION "unknown")
endif()

# Add compile definitions for note-c
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    NOTE_C_TEST=0
    NOTE_ESPIDF_VER="${COMPONENT_VERSION}"
)
