#
# CMake Toolchain file for crosscompiling on ARM.
#
# Target operating system name.
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(CMAKE_CROSSCOMPILING TRUE)

# Name of C compiler.
set(CMAKE_C_COMPILER "/NXP/S32DS.3.4/S32DS/build_tools/gcc_v10.2/gcc-10.2-arm64-linux/bin/aarch64-linux-gnu-gcc-10.2.0")
set(CMAKE_CXX_COMPILER "/NXP/S32DS.3.4/S32DS/build_tools/gcc_v10.2/gcc-10.2-arm64-linux/bin/aarch64-linux-gnu-g++")

# Where to look for the target environment. (More paths can be added here)
set(CMAKE_FIND_ROOT_PATH /NXP/S32DS.3.4/S32DS/build_tools/gcc_v10.2/gcc-10.2-arm64-linux)
set(CMAKE_INCLUDE_PATH  /NXP/S32DS.3.4/S32DS/build_tools/gcc_v10.2/gcc-10.2-arm64-linux/include)
set(CMAKE_LIBRARY_PATH  /NXP/S32DS.3.4/S32DS/build_tools/gcc_v10.2/gcc-10.2-arm64-linux/lib)
set(CMAKE_PROGRAM_PATH  /NXP/S32DS.3.4/S32DS/build_tools/gcc_v10.2/gcc-10.2-arm64-linux/bin)

# Adjust the default behavior of the FIND_XXX() commands:
# search programs in the host environment only.
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)

# Search headers and libraries in the target environment only.
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
