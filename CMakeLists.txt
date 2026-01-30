cmake_minimum_required(VERSION 3.22)

# ============================
# Project
# ============================
set(CMAKE_PROJECT_NAME Kawashiro_Frame_G474)

project(${CMAKE_PROJECT_NAME} C ASM)

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
set(CMAKE_BUILD_TYPE Release)
# ============================
# Build type
# ============================
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

# ============================
# Language standard
# ============================
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# ============================
# clangd support
# ============================
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ============================
# Executable
# ============================
add_executable(${CMAKE_PROJECT_NAME})

# ============================
# STM32CubeMX generated code
# ============================
add_subdirectory(cmake/stm32cubemx)

target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
)

# ============================
# User sources
# ============================
file(GLOB_RECURSE USER_SOURCES
    Project/*.c
)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${USER_SOURCES}
    CMSIS-DSP/1.16.2/Source/MatrixFunctions/MatrixFunctions.c
)

# ============================
# Include directories
# ============================
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE

    # -------- CubeMX Core --------
    Core/Inc

    # -------- CMSIS --------
    CMSIS-DSP/1.16.2/ComputeLibrary/Include
    CMSIS-DSP/1.16.2/PrivateInclude
    Drivers/CMSIS/Device/ST/STM32G4xx/Include
    Drivers/CMSIS/Include
    CMSIS-DSP/1.16.2/Include
    # -------- HAL --------
    Drivers/STM32G4xx_HAL_Driver/Inc
    Drivers/STM32G4xx_HAL_Driver/Inc/Legacy

    # -------- USB --------
    USB_Device/App
    USB_Device/Target

    # -------- User Project --------
    Project
    Project/Algorithm_Drivers
    Project/BSP
    Project/Commnuicate_Drivers
    Project/Hardware_Drivers
    Project/Robot_Application
    Project/UI
)

# ============================
# Compile definitions
# ============================
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    ARM_MATH_CM4
    STM32G474xx
)

# ============================
# Fix toolchain quirks
# ============================
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)