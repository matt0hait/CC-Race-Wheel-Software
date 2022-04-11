cmake_minimum_required(VERSION 3.20)

# Definitions
set(PROJECT_NAME CC-Race-Wheel-Software)
# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

project(${PROJECT_NAME} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/bin")

list(APPEND SOURCE_FILES
        main.c
        include/tusb_config/usb_descriptors.c
        include/tusb_config/tusb_config.h
        )

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.3.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.3.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

# Initialize the SDK
pico_sdk_init()

add_compile_options(-Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        -Wno-maybe-uninitialized
        )
include_directories(include/utils/tusb_config/ PUBLIC)
add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} pico_stdlib tinyusb_device tinyusb_board)
pico_add_extra_outputs(${PROJECT_NAME})