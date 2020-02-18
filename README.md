# Block Matching
This is an implementation of  [A comparison of block-matching motion estimation algorithms](https://ieeexplore.ieee.org/document/6398002) paper.

## Dependencies
* OpenCV 4.2.0
* VTK 8.2.0
* GLEW 2.1.0

## Build
A working CMake installation is required

Create a build directory in the root directory:
```bash
mkdir build 
```

For generating Linux Release Makefile:
```bash
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
```

For generating Linux Debug Makefile:
```bash
cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug
```

Then type
```bash
make -j
```
