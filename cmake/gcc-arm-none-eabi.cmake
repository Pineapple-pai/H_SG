set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

set(CMAKE_C_COMPILER_ID GNU)
set(CMAKE_CXX_COMPILER_ID GNU)

# Resolve the ARM GNU toolchain explicitly so post-build tools do not depend on PATH.
set(TOOLCHAIN_PREFIX arm-none-eabi-)
set(ARM_GNU_TOOLCHAIN_BIN "" CACHE PATH "Path to the ARM GNU toolchain bin directory")

if(NOT ARM_GNU_TOOLCHAIN_BIN)
    find_program(ARM_NONE_EABI_GCC
        NAMES ${TOOLCHAIN_PREFIX}gcc
        HINTS
            "C:/arm-gnu-toolchain-14.2.rel1-mingw-w64-i686-arm-none-eabi/bin"
            "C:/arm-gnu-toolchain-13.3.rel1-mingw-w64-i686-arm-none-eabi/bin"
            "D:/arm-gnu-toolchain-14.2.rel1-mingw-w64-i686-arm-none-eabi/bin"
            "D:/arm-gnu-toolchain-13.3.rel1-mingw-w64-i686-arm-none-eabi/bin"
    )

    if(ARM_NONE_EABI_GCC)
        get_filename_component(ARM_GNU_TOOLCHAIN_BIN "${ARM_NONE_EABI_GCC}" DIRECTORY)
    else()
        set(ARM_GNU_TOOLCHAIN_BIN "")
    endif()
endif()

if(ARM_GNU_TOOLCHAIN_BIN)
    file(TO_CMAKE_PATH "${ARM_GNU_TOOLCHAIN_BIN}" ARM_GNU_TOOLCHAIN_BIN)
    set(CMAKE_C_COMPILER   "${ARM_GNU_TOOLCHAIN_BIN}/${TOOLCHAIN_PREFIX}gcc")
    set(CMAKE_ASM_COMPILER "${CMAKE_C_COMPILER}")
    set(CMAKE_CXX_COMPILER "${ARM_GNU_TOOLCHAIN_BIN}/${TOOLCHAIN_PREFIX}g++")
    set(CMAKE_LINKER       "${ARM_GNU_TOOLCHAIN_BIN}/${TOOLCHAIN_PREFIX}g++")
    set(CMAKE_AR           "${ARM_GNU_TOOLCHAIN_BIN}/${TOOLCHAIN_PREFIX}ar")
    set(CMAKE_RANLIB       "${ARM_GNU_TOOLCHAIN_BIN}/${TOOLCHAIN_PREFIX}ranlib")
    set(CMAKE_NM           "${ARM_GNU_TOOLCHAIN_BIN}/${TOOLCHAIN_PREFIX}nm")
    set(CMAKE_OBJCOPY      "${ARM_GNU_TOOLCHAIN_BIN}/${TOOLCHAIN_PREFIX}objcopy")
    set(CMAKE_SIZE         "${ARM_GNU_TOOLCHAIN_BIN}/${TOOLCHAIN_PREFIX}size")
else()
    set(CMAKE_C_COMPILER   ${TOOLCHAIN_PREFIX}gcc)
    set(CMAKE_ASM_COMPILER ${CMAKE_C_COMPILER})
    set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}g++)
    set(CMAKE_LINKER       ${TOOLCHAIN_PREFIX}g++)
    set(CMAKE_AR           ${TOOLCHAIN_PREFIX}ar)
    set(CMAKE_RANLIB       ${TOOLCHAIN_PREFIX}ranlib)
    set(CMAKE_NM           ${TOOLCHAIN_PREFIX}nm)
    set(CMAKE_OBJCOPY      ${TOOLCHAIN_PREFIX}objcopy)
    set(CMAKE_SIZE         ${TOOLCHAIN_PREFIX}size)
endif()

set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(CMAKE_C_COMPILER_FORCED TRUE)
set(CMAKE_CXX_COMPILER_FORCED TRUE)
set(CMAKE_C_COMPILER_WORKS TRUE CACHE INTERNAL "")
set(CMAKE_CXX_COMPILER_WORKS TRUE CACHE INTERNAL "")

# MCU specific flags
set(TARGET_FLAGS "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard ")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${TARGET_FLAGS}")
set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp -MMD -MP")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -fdata-sections -ffunction-sections")

set(CMAKE_C_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_C_FLAGS_RELEASE "-Os -g0")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_CXX_FLAGS_RELEASE "-Os -g0")

set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics")

set(CMAKE_EXE_LINKER_FLAGS "${TARGET_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T \"${CMAKE_SOURCE_DIR}/STM32F407XX_FLASH.ld\"")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --specs=nano.specs")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--print-memory-usage")
set(TOOLCHAIN_LINK_LIBRARIES "m")
