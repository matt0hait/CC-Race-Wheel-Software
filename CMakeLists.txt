cmake_minimum_required(VERSION 3.20)

# Definitions
set(PROJECT_NAME CC-Race-Wheel-Software)
# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

project(${PROJECT_NAME} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/bin")
set(FREERTOS_SRC_DIRECTORY "${CMAKE_SOURCE_DIR}/include/FreeRTOS-Kernel")
set(FREERTOS_CFG_DIRECTORY "${CMAKE_SOURCE_DIR}/include/FreeRTOS")
set(TRACE_RECORDER_SRC_DIRECTORY "${CMAKE_SOURCE_DIR}/include/TraceRecorder")

list(APPEND SOURCE_FILES
        main.c
        include/tusb_config/usb_descriptors.c
        include/tusb_config/tusb_config.h
        include/tusb_config/custom_gamepad.h
        )

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.3.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.3.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

add_compile_definitions(APP_NAME="${APP_1_NAME}")
add_compile_definitions(APP_VERSION="${APP_1_VERSION_NUMBER}")
add_compile_definitions(BUILD_NUM=${BUILD_NUMBER})

# Initialize the SDK
pico_sdk_init()

# Add FreeRTOS as a library
add_library(FreeRTOS STATIC
        ${FREERTOS_SRC_DIRECTORY}/event_groups.c
        ${FREERTOS_SRC_DIRECTORY}/list.c
        ${FREERTOS_SRC_DIRECTORY}/queue.c
        ${FREERTOS_SRC_DIRECTORY}/stream_buffer.c
        ${FREERTOS_SRC_DIRECTORY}/tasks.c
        ${FREERTOS_SRC_DIRECTORY}/timers.c
        ${FREERTOS_SRC_DIRECTORY}/portable/MemMang/heap_3.c
        ${FREERTOS_SRC_DIRECTORY}/portable/GCC/ARM_CM0/port.c
        )
# Build FreeRTOS
target_include_directories(FreeRTOS PUBLIC
        ${FREERTOS_CFG_DIRECTORY}/
        ${FREERTOS_SRC_DIRECTORY}/include
        ${FREERTOS_SRC_DIRECTORY}/portable/GCC/ARM_CM0
        ${FREERTOS_SRC_DIRECTORY}/portable/GCC/MemMang
        )


add_library(TraceRecorder STATIC
        ${TRACE_RECORDER_SRC_DIRECTORY}/trcAssert.c
        ${TRACE_RECORDER_SRC_DIRECTORY}/trcCounter.c
        ${TRACE_RECORDER_SRC_DIRECTORY}/trcDiagnostics.c
        ${TRACE_RECORDER_SRC_DIRECTORY}/trcEntryTable.c
        ${TRACE_RECORDER_SRC_DIRECTORY}/trcError.c
        ${TRACE_RECORDER_SRC_DIRECTORY}/trcEvent.c
        ${TRACE_RECORDER_SRC_DIRECTORY}/trcEventBuffer.c
        ${TRACE_RECORDER_SRC_DIRECTORY}/trcExtension.c
        ${TRACE_RECORDER_SRC_DIRECTORY}/trcHardwarePort.c
        ${TRACE_RECORDER_SRC_DIRECTORY}/trcHeap.c
        ${TRACE_RECORDER_SRC_DIRECTORY}/trcInternalEventBuffer.c
        ${TRACE_RECORDER_SRC_DIRECTORY}/trcInterval.c
        ${TRACE_RECORDER_SRC_DIRECTORY}/trcISR.c
        ${TRACE_RECORDER_SRC_DIRECTORY}/trcKernelPort.c
        ${TRACE_RECORDER_SRC_DIRECTORY}/trcMultiCoreEventBuffer.c
        ${TRACE_RECORDER_SRC_DIRECTORY}/trcObject.c
        ${TRACE_RECORDER_SRC_DIRECTORY}/trcPrint.c
        ${TRACE_RECORDER_SRC_DIRECTORY}/trcSnapshotRecorder.c
        ${TRACE_RECORDER_SRC_DIRECTORY}/trcStackMonitor.c
        ${TRACE_RECORDER_SRC_DIRECTORY}/trcStateMachine.c
        ${TRACE_RECORDER_SRC_DIRECTORY}/trcStaticBuffer.c
        ${TRACE_RECORDER_SRC_DIRECTORY}/trcStreamingRecorder.c
        ${TRACE_RECORDER_SRC_DIRECTORY}/trcString.c
        ${TRACE_RECORDER_SRC_DIRECTORY}/trcTask.c
        ${TRACE_RECORDER_SRC_DIRECTORY}/trcTimestamp.c
        ${TRACE_RECORDER_SRC_DIRECTORY}/streamports/RingBuffer/trcStreamPort.c
        )
target_include_directories(TraceRecorder PUBLIC
        ${TRACE_RECORDER_SRC_DIRECTORY}/
        ${TRACE_RECORDER_SRC_DIRECTORY}/config
        ${TRACE_RECORDER_SRC_DIRECTORY}/include
        ${TRACE_RECORDER_SRC_DIRECTORY}/extras
        ${TRACE_RECORDER_SRC_DIRECTORY}/streamports
        )

add_compile_options(-Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        -Wno-maybe-uninitialized
        )
include_directories(    include/tusb_config/
                        include/FreeRTOS/
                        include/TraceRecorder/
                        include/TraceRecorder/include
                        include/TraceRecorder/config
                        include/TraceRecorder/streamports/
                        include/TraceRecorder/streamports/RingBuffer/include/
                        include/TraceRecorder/streamports/RingBuffer/config/ PUBLIC
        )
add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(TraceRecorder FreeRTOS)
target_link_libraries(${PROJECT_NAME} pico_stdlib FreeRTOS TraceRecorder tinyusb_device tinyusb_board)
pico_add_extra_outputs(${PROJECT_NAME})