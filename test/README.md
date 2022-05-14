# Tests

This directory is a catkin workspace for testing the root repository's tools against real ROS packages.

# Build Instructions

First we configure our compiler options via Cmake args:

```bash
# tell catkin to tell CMake to use clang for compilation and our clang plugin
catkin config --cmake-args -DCMAKE_C_COMPILER=$(which clang) -DCMAKE_CXX_COMPILER=$(which clang++) -DCMAKE_C_FLAGS="-Xclang -load -Xclang /workspace/build/libFindROSPrimitivesPlugin.so -Xclang -add-plugin -Xclang find-ros-primitives" -DCMAKE_CXX_FLAGS="-Xclang -load -Xclang /workspace/build/libFindROSPrimitivesPlugin.so -Xclang -add-plugin -Xclang find-ros-primitives"
```

```bash
# to run custom clang plugin as part of the build process
catkin clean
catkin build 
```
