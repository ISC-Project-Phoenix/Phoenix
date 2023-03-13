# Phoenix
Main repo for the ROS side of Project Phoenix

## Building

Before building phoenix, make sure to set:
```export GZ_VERSION=garden```

This sets ros-gz to target gazebo garden.

## Launching

### Sim

#### Just sim

`ros2 launch phoenix_gazebo common.launch.py`

#### Data collection

`ros2 launch phoenix_gazebo data_logger.launch.py`
