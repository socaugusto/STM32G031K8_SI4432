cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 23)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME stmg031k8)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ./Core/Src/system_stm32g0xx.c
    ./Core/Src/sysmem.c
    ./Core/Src/syscalls.c
    ./startup_stm32g031xx.s
    ./Core/Src/main.c
    ./Core/Src/gpio.c
    ./Core/Src/dma.c
    ./Core/Src/rtc.c
    ./Core/Src/spi.c
    ./Core/Src/tim.c
    ./Core/Src/usart.c
    ./Core/Src/stm32g0xx_it.c
    ./Core/Src/stm32g0xx_hal_msp.c

    ./Core/Src/communication.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

add_custom_target(${CMAKE_PROJECT_NAME}_Flash_1
    COMMAND openocd -f interface/stlink.cfg -f target/stm32g0x.cfg -c "hla_serial 066CFF3933354D5043165420" -c "program stmg031k8.elf verify reset exit"
    DEPENDS ${CMAKE_PROJECT_NAME}
)

add_custom_target(${CMAKE_PROJECT_NAME}_Flash_2
    COMMAND openocd -f interface/stlink.cfg -f target/stm32g0x.cfg -c "hla_serial 066CFF313541483043202339" -c "program stmg031k8.elf verify reset exit"
    DEPENDS ${CMAKE_PROJECT_NAME}
)
