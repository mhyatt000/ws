Add a custom Python node#
As ros works with custom nodes, let's add a custom node to our project.


pixi run ros2 pkg create --build-type ament_python --destination-directory src --node-name my_node my_package
To build the package we need some more dependencies:


pixi add colcon-common-extensions "setuptools<=58.2.0"
Add the created initialization script for the ros workspace to your manifest file.

Then run the build command


pixi run colcon build


Build a C++ node#
To build a C++ node you need to add the ament_cmake and some other build dependencies to your manifest file.


pixi add ros-humble-ament-cmake-auto compilers pkg-config cmake ninja
Now you can create a C++ node with the following command


pixi run ros2 pkg create --build-type ament_cmake --destination-directory src --node-name my_cpp_node my_cpp_package
Now you can build it again and run it with the following commands


# Passing arguments to the build command to build with Ninja, add them to the manifest if you want to default to ninja.
pixi run build --cmake-args -G Ninja
pixi run ros2 run my_cpp_package my_cpp_node

