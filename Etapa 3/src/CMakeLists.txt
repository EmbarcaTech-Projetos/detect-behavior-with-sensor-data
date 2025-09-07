# Generated Cmake Pico project file

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
set(picotoolVersion 2.2.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(src C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(src 
            src.c 
            drivers/mpu9250/driver_mpu9250.c
            drivers/mlx90614/driver_mlx90164.c
            apps/madgwick/madgwick.c
            apps/network/network.c
)

pico_set_program_name(src "src")
pico_set_program_version(src "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(src 0)
pico_enable_stdio_usb(src 1)

# Add the standard library to the build
target_link_libraries(src
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(src PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/drivers/mpu9250
        ${CMAKE_CURRENT_LIST_DIR}/drivers/mlx90614
        ${CMAKE_CURRENT_LIST_DIR}/apps/madgwick
        ${CMAKE_CURRENT_LIST_DIR}/apps/network
)

# Add any user requested libraries
target_link_libraries(src 
        hardware_i2c
        pico_cyw43_arch_lwip_threadsafe_background
        )

pico_add_extra_outputs(src)

