cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(Sentinel C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

add_executable(Sentinel # entry point and generic routines
                        src/main.cpp src/console.cpp src/hdd.cpp src/endec.cpp src/crc.cpp src/sd.cpp src/dos.cpp src/commands.cpp
                        # low level formats
                        src/llf/wd.cpp src/llf/omti.cpp src/llf/xebec_adaptec.cpp src/llf/hdc9224.cpp src/llf/sm1040.cpp
                        )

pico_set_program_name(Sentinel "Sentinel")
pico_add_extra_outputs(Sentinel)

# SD library
add_subdirectory(src/3rdparty/no-OS-FatFS-SD-SDIO-SPI-RPi-Pico/src build)

# PIO
pico_generate_pio_header(Sentinel ${CMAKE_CURRENT_LIST_DIR}/src/pio/sampler.pio)
pico_generate_pio_header(Sentinel ${CMAKE_CURRENT_LIST_DIR}/src/pio/writer.pio)
pico_generate_pio_header(Sentinel ${CMAKE_CURRENT_LIST_DIR}/src/pio/wclock_test.pio)

# Output over UART/USB
pico_enable_stdio_uart(Sentinel 0)
pico_enable_stdio_usb(Sentinel 1)

# Add the standard library to the build
target_link_libraries(Sentinel  # RPi
                                pico_stdlib pico_multicore pico_rand hardware_adc hardware_pio hardware_gpio hardware_dma hardware_clocks hardware_flash
                                # SD library
                                no-OS-FatFS-SD-SDIO-SPI-RPi-Pico
                                )

# Add the standard include files to the build
target_include_directories(Sentinel PRIVATE 
                                    ${CMAKE_CURRENT_LIST_DIR}
                                    ${CMAKE_CURRENT_LIST_DIR}/src
                                    ${CMAKE_CURRENT_LIST_DIR}/src/pio
                                    ${CMAKE_CURRENT_LIST_DIR}/src/llf
)

# requires -O3 and a sysclk of >=250MHz
target_compile_options(Sentinel PRIVATE -O3)
target_compile_definitions(Sentinel PRIVATE PICO_EMBED_XIP_SETUP=1)
target_compile_definitions(bs2_default PRIVATE PICO_FLASH_SPI_CLKDIV=2)

