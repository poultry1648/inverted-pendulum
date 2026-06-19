cmake_minimum_required(VERSION 3.13)

# Pull in the Pico SDK (vendored under lib/pico-sdk). This must come before
# the project() call so the SDK can inject its toolchain settings. An external
# PICO_SDK_PATH (env or -D) still wins if you'd rather use a system-wide SDK.
if(NOT DEFINED PICO_SDK_PATH AND NOT DEFINED ENV{PICO_SDK_PATH})
    set(PICO_SDK_PATH ${CMAKE_CURRENT_LIST_DIR}/lib/pico-sdk)
endif()
include(pico_sdk_import.cmake)

project(skr_pico_fw C CXX ASM)

# SKR Pico = RP2040. The stock "pico" board profile (2MB flash) matches it.
set(PICO_BOARD pico CACHE STRING "Target board")

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(skr_pico_fw
    src/main.c
    src/tmc2209.c
)

target_link_libraries(skr_pico_fw
    pico_stdlib   # gpio, time, stdio, etc.
    hardware_uart # TMC2209 driver config
)

# Console over the USB-CDC port (the board's USB-C). Set the second arg to 1
# to also mirror stdio onto UART0, or swap if you wire a serial adapter.
pico_enable_stdio_usb(skr_pico_fw 1)
pico_enable_stdio_uart(skr_pico_fw 0)

# Produce .uf2 / .elf / .bin / .hex / map + disassembly.
pico_add_extra_outputs(skr_pico_fw)
