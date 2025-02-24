cmake_minimum_required(VERSION 3.12)

set(PICO_BOARD pico2)
# set(PICO_DEOPTIMIZED_DEBUG ON)
set(PICO_FLASH_SIZE_BYTES "2 * 1024 * 1024")

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

include(FreeRTOS_Kernel.cmake)

project(project C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# 添加FreeRTOS源文件
add_executable(project
        main.c
)

# 添加FreeRTOS头文件路径
target_include_directories(project PRIVATE
        Core/Include
)

target_link_libraries(project
        pico_async_context_freertos
        FreeRTOS-Kernel-Heap4
        pico_stdlib
        pico_multicore
        pico_sync
        hardware_pio
        hardware_dma
        hardware_spi
)

pico_enable_stdio_usb(project 0)
pico_enable_stdio_uart(project 0)

set_property(TARGET project APPEND_STRING PROPERTY LINK_FLAGS "-Wl,--gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map")

pico_add_extra_outputs(project)
