# Tests

This directory is a catkin workspace for testing the root repository's tools against real ROS packages.

# Build Instructions

First, we need to build our main project plugin. If you haven't already done so, execute the following from the root of this repository (outside the test folder):

```bash
mkdir build
cd build
cmake ../
make
cd ../
```

Next, we configure our compiler options via Cmake args:

```bash
# navigate to the test directory
cd test

# tell catkin to tell CMake to use clang for compilation and our clang plugin
catkin config --cmake-args -DCMAKE_C_COMPILER=$(which clang) -DCMAKE_CXX_COMPILER=$(which clang++) -DCMAKE_C_FLAGS="-Xclang -load -Xclang /workspace/build/libFindROSPrimitivesPlugin.so -Xclang -add-plugin -Xclang find-ros-primitives" -DCMAKE_CXX_FLAGS="-Xclang -load -Xclang /workspace/build/libFindROSPrimitivesPlugin.so -Xclang -add-plugin -Xclang find-ros-primitives"
```

```bash
# to run custom clang plugin as part of the build process
catkin clean 
catkin build 

# if everything succeeds you should see some of the following lines:
remark: Found ros::Publisher at 48:3 in /opt/ros/noetic/include/ros/publisher.h
```
