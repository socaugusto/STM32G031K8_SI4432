cmake_minimum_required(VERSION 3.22)

add_library(si4432
    ./SI4432/src/si4432.c
    ./SI4432/src/si4432_io_hal.c
)

target_include_directories(si4432
    PUBLIC
        ./SI4432/inc/
        ../Core/Inc/
)

target_link_libraries(si4432
    PRIVATE
        stm32cubemx
)