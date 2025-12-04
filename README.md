# isaac-sim-yarp-bridge
Connection between the Isaac Sim ROS2 bridge and YARP interfaces.

The repo is composed of two main parts:
1. A set of YARP devices that connect to the Isaac Sim ROS2 bridge topics and services.
2. A set of Python scripts to launch inside Isaac Sim to create an action graph that publishes and subscribes to the relevant ROS2 topics and services.

## C++ devices installation

### Dependencies
The C++ dependencies can be installed via ``conda`` using the following command:
```bash
conda install -c conda-forge -c robostack-jazzy cxx-compiler cmake pkg-config make ninja colcon-common-extensions catkin_tools rosdep ros-jazzy-desktop yarp catch2 ergocub-software
```
### Build and install
To build and install the YARP devices, clone the repository. Then, from the root of the repository, run:
```bash
mkdir build
cd build
cmake .. -GNinja -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=<install_path>
cmake --build . --target install
```
Replace `<install_path>` with the desired installation path.

## Python scripts installation
The scripts in the [`scripts`](./scripts) folder can be used inside Isaac Sim to create an action graph that connects to the ROS2 bridge topics and services.
To use them, simply copy their content in a script editor inside Isaac Sim and run them.
Some of them require the location of the other scripts. Hence, it is necessary to set the following environment variable
```bash
export ISAAC_SIM_YARP_BRIDGE_PATH=<path_to_the_cloned_repository>
```
