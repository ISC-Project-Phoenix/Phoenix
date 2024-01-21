# Phoenix

Main repo for the ROS side of Project Phoenix

### Dependencies

All dependencies for phnx are described in cmake and package files.
This. This means that if you are missing a package somewhere, it will error for you.

To ensure you have all source nodes, be sure to `vcs import` `pheonix.repos`.

If using Webots, you must install webots separately.

## Launching

### Sim

#### Just sim

By default, webots will be launched:
`ros2 launch phoenix_gazebo common.launch.py`
