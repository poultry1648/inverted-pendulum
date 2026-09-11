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
    src/motion.c  # step generation, position, limits, safety
    src/control.c # core 1 real-time loop + inter-core plumbing
)

target_link_libraries(skr_pico_fw
    pico_stdlib    # gpio, time, stdio, etc.
    pico_multicore # launch + synchronise the core 1 control loop
    pico_util      # core-safe command queue
    hardware_uart  # TMC2209 driver config
    hardware_adc   # TH0 potentiometer readout
)

# Console over the USB-CDC port (the board's USB-C). Set the second arg to 1
# to also mirror stdio onto UART0, or swap if you wire a serial adapter.
pico_enable_stdio_usb(skr_pico_fw 1)
pico_enable_stdio_uart(skr_pico_fw 0)

# Produce .uf2 / .elf / .bin / .hex / map + disassembly.
pico_add_extra_outputs(skr_pico_fw)

# Convenience for the VS Code CMake Tools "Build" button: after the SDK has
# generated skr_pico_fw.uf2, copy it onto a mounted RPI-RP2 BOOTSEL drive. The
# script is a no-op when the board is not in boot mode, so it never fails a
# build. Must stay after pico_add_extra_outputs() so the .uf2 already exists.
add_custom_command(TARGET skr_pico_fw POST_BUILD
    COMMAND bash "${CMAKE_CURRENT_SOURCE_DIR}/scripts/flash_uf2.sh"
            "${CMAKE_CURRENT_BINARY_DIR}/skr_pico_fw.uf2"
    COMMENT "Copying skr_pico_fw.uf2 to the RPI-RP2 drive (if mounted)"
    VERBATIM
)
