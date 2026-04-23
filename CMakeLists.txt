cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME SDolve_Wind_XS-WSDS01)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C CXX ASM)

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
    # User C sources
    Core/Src/rs485_wind.c
    Core/Src/serial_debug.c
    
    # User C++ sources - Main application
    Core/Src/main.cpp
    
    # NMEA2000 library sources
    Core/Libs/NMEA2000-master/NMEA2000.cpp
    Core/Libs/NMEA2000-master/N2kMsg.cpp
    Core/Libs/NMEA2000-master/N2kMessages.cpp
    Core/Libs/NMEA2000-master/N2kStream.cpp
    Core/Libs/NMEA2000-master/N2kTimer.cpp
    Core/Libs/NMEA2000-master/N2kGroupFunction.cpp
    Core/Libs/NMEA2000-master/N2kGroupFunctionDefaultHandlers.cpp
    Core/Libs/NMEA2000-master/N2kDeviceList.cpp
    Core/Libs/NMEA2000-master/N2kMaretron.cpp
    Core/Libs/NMEA2000-master/Seasmart.cpp
    
    # NMEA2000 STM32 adapter
    Core/Libs/NMEA2000_STM32-master/NMEA2000_STM32.cpp
    Core/Libs/NMEA2000_STM32-master/N2kStream_STM32.cpp
    
    # STM32 CAN driver
    Core/Libs/STM32_CAN-main/STM32_CAN.cpp
    Core/Libs/STM32_CAN-main/scheduler.cpp
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # User includes
    Core/Inc
    
    # NMEA2000 library includes
    Core/Libs/NMEA2000-master
    Core/Libs/NMEA2000_STM32-master
    Core/Libs/STM32_CAN-main
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
