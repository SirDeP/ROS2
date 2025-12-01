# README

Here is a short readme about the package usb camera subscriber package

This package will use raw camera data provided by the `usb_cam` package, tranform it using opencv and display it in rviz

## Configuration

To configure the package you can edit the launch file located inside of the `vw-assign2-2025-usb-cam-sub-pkg` package folder: [usb_cam.launch.xml](src/vw-assign2-2025-usb-cam-sub-pkg/launch/usb_cam.launch.xml)

Launching using this file preconfigures certain values into the parameters, but it will also launch a parameter configuration window where changes have effect on the running program.

The runtime changes only work for the `usb_cam_reader_node` and not for the `usb_cam` driver. The video device thus has to be configured before launching the program

## Build

To build the packages run the `colcon build` command in the root directory of the workspace.

## Run

To run the node use the `ros2 launch` command.
- `ros2 launch vw-assign2-2025-usb-cam-sub-pkg usb_cam.launch.xml`
