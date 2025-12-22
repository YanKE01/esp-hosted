# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

file(READ "main/include/esp_fw_version.h" HEADER_CONTENTS)
set(PROJECT_VERSION_REGEX "#define[ \t]+PROJECT_NAME[ \t]+\"([A-Z]+)\"[ \t\n]+#define[ \t]+PROJECT_VERSION_MAJOR_1[ \t]+([0-9]+)[ \t\n]+#define[ \t]+PROJECT_VERSION_MAJOR_2[ \t]+([0-9]+)[ \t\n]+#define[ \t]+PROJECT_VERSION_MINOR[ \t]+([0-9]+)[ \t\n]+#define[ \t]+PROJECT_REVISION_PATCH_1[ \t]+([0-9]+)[ \t\n]+#define[ \t]+PROJECT_REVISION_PATCH_2[ \t]+([0-9]+)")
string(REGEX MATCH "${PROJECT_VERSION_REGEX}" PROJECT_VERSION_MATCH "${HEADER_CONTENTS}")

if(PROJECT_VERSION_MATCH)
    set(PROJECT_NAME "${CMAKE_MATCH_1}")
    set(PROJECT_VERSION_MAJOR_1 "${CMAKE_MATCH_2}")
    set(PROJECT_VERSION_MAJOR_2 "${CMAKE_MATCH_3}")
    set(PROJECT_VERSION_MINOR "${CMAKE_MATCH_4}")
    set(PROJECT_REVISION_PATCH_1 "${CMAKE_MATCH_5}")
    set(PROJECT_REVISION_PATCH_2 "${CMAKE_MATCH_6}")

    # Construct project version
    set(PROJECT_VER "${PROJECT_NAME}-${PROJECT_VERSION_MAJOR_1}.${PROJECT_VERSION_MAJOR_2}.${PROJECT_VERSION_MINOR}.${PROJECT_REVISION_PATCH_1}.${PROJECT_REVISION_PATCH_2}")

    message(*************************************************************************************)
    message("                    Building ESP-Hosted Firmware version :: ${PROJECT_VER} ")
    message(*************************************************************************************)
else()
    message(FATAL_ERROR "Failed to read project version from main/include/esp_fw_version.h")
endif()

set(PROJECT_VER "${PROJECT_VER}")

# Read sdkconfig before project() to set environment variable for component manager
set(SDKCONFIG_FILE "${CMAKE_SOURCE_DIR}/sdkconfig")
set(HOSTED_NG_ENABLE_TINYUSB "False")
if(EXISTS "${SDKCONFIG_FILE}")
    file(STRINGS "${SDKCONFIG_FILE}" SDKCONFIG_LINES)
    list(FIND SDKCONFIG_LINES "CONFIG_ESP_USB_HOST_INTERFACE=y" USB_FOUND)
    if(USB_FOUND GREATER_EQUAL 0)
        set(HOSTED_NG_ENABLE_TINYUSB "True")
    endif()
endif()

# Set environment variable for component manager (must be before project())
set(ENV{HOSTED_NG_ENABLE_TINYUSB} ${HOSTED_NG_ENABLE_TINYUSB})

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
set(EXCLUDE_COMPONENTS wpa_supplicant)
project(network_adapter)
idf_build_set_property(COMPILE_OPTIONS "-fdiagnostics-color=always" APPEND)
