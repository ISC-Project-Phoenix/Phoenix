# Phoenix
Main repo for the ROS side of Project Phoenix

## Building

Before building phoenix, make sure to set:
```export GZ_VERSION=garden```

This sets ros-gz to target gazebo garden.

### Dependencies

All dependencies for phnx are described in cmake and package files.
This. This means that if you are missing a package somewhere, it will error for you.

To ensure you have all source nodes, be sure to `vcs import` `pheonix.repos`.

## Launching

### Sim

#### Just sim

`ros2 launch phoenix_gazebo common.launch.py`

#### Data collection

`ros2 launch phoenix_gazebo data_logger.launch.py`
