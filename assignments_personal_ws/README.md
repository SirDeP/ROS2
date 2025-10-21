# README
Here is a short readme about the three packages with different implementations of adding objects to a RVIZ environment.

They use fixed, mesh publisher and static transform methods respectively and can be identified by their name

Collision using simple geometry was added to the robot and in the fixed and static transform packages also to the coin. Collision for the board was left on mesh as the STL's have complex geometry.

With the mesh publisher package only the robot has collisions as i could find no way to add colision to the mesh markers themselves.

## Configuration

- Fixed package:
    - To configure the location of the objects for this node the locations need to be changed in the URDF xacro file directly.
- Mesh and Static transform packages:
    - To configure the location of the objects for this node the specific parameters can be used.
    - Parameters are configured in the launch xml (`display.launch.xml`) located in the respective launch folder of the package that you want to configure.

## Build
To build the packages run the `colcon build` command in the root directory of the workspace.

## Run
To run the nodes use the `ros2 launch` command.

Robot Only Package
- `ros2 launch vw-assign1-2025-parol6-robot-pkg display.launch.xml`

Fixed Package:
- `ros2 launch vw-assign1-2025-parol6-fixed-pkg display.launch.xml`

Mesh Package:
- `ros2 launch vw-assign1-2025-parol6-mesh-pkg display.launch.xml`

Static Transform Package:
- `ros2 launch vw-assign1-2025-parol6-static-pkg display.launch.xml`

