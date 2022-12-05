# build step

# - build directory
mkdir bld_dir
cd bld_dir

# - prepare cmake for windows or linux
cmake ../ .
# - prepare cmake for arm build
cmake ../ . -DARCH=S32R45 -DCMAKE_TOOLCHAIN_FILE:PATH="toolchain_arm.cmake"

# build
cmake --build . [--config Release]


### etc....
scp libtitrk.so root@192.168.137.15:/home/root

### lib directory
- titrk.dll/exp/lib file is windows file
- libtitrk.so file is linux file(intel linux, not s32r45)


