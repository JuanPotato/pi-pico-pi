cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(pi_pico_pi C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_executable(pi_pico_pi)

pico_generate_pio_header(pi_pico_pi ${CMAKE_CURRENT_LIST_DIR}/hyperram.pio)

target_sources(pi_pico_pi PRIVATE pi_pico_pi.c)

target_link_libraries(pi_pico_pi PRIVATE
    pico_stdlib
    hardware_pio
    hardware_dma
)

pico_enable_stdio_uart(pi_pico_pi 1)
pico_add_extra_outputs(pi_pico_pi)
