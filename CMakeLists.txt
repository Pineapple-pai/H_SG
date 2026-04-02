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
set(CMAKE_CXX_EXTENSIONS OFF)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME SG_CHASSIS_C_2024_11_5)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM, C and C++ languages
enable_language(C CXX ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Collect legacy user sources that were previously built from the MDK project.
file(GLOB_RECURSE USER_SOURCES
    "${CMAKE_SOURCE_DIR}/User/*.c"
    "${CMAKE_SOURCE_DIR}/User/*.cpp"
    "${CMAKE_SOURCE_DIR}/User/*.s"
    "${CMAKE_SOURCE_DIR}/User/*.S"
)

file(GLOB_RECURSE USER_HEADERS
    "${CMAKE_SOURCE_DIR}/User/*.h"
    "${CMAKE_SOURCE_DIR}/User/*.hpp"
)

# The legacy tree currently contains a duplicated StateWatch implementation.
# Keep the BSP root variant, which matches the include paths used by the code.
list(FILTER USER_SOURCES EXCLUDE REGEX ".*/User/BSP/Common/StateWatch/state_watch\\.cpp$")

set(USER_INCLUDE_DIRS)

foreach(header ${USER_HEADERS})
    get_filename_component(header_dir "${header}" DIRECTORY)
    list(APPEND USER_INCLUDE_DIRS "${header_dir}")
endforeach()

list(REMOVE_DUPLICATES USER_INCLUDE_DIRS)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${USER_SOURCES}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${USER_INCLUDE_DIRS}
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    ARM_MATH_CM4
    __FPU_PRESENT=1U
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-fno-use-cxa-atexit>
)

if(CMAKE_OBJCOPY)
    add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex"
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin"
        COMMENT "Generating HEX and BIN images"
    )
endif()
