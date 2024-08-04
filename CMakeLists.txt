cmake_minimum_required(VERSION 3.13)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

project(main C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

include_directories(pico_i2c_common)
add_subdirectory(pico_i2c_common)

add_executable(main
        main.c
)

target_link_libraries(main 
    pico_stdlib
    pico_multicore 
    hardware_irq 
    hardware_i2c
    pico_i2c_common
)


pico_add_extra_outputs(main)
