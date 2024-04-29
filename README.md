# Phoenix

Main repo for the ROS side of Project Phoenix

### Dependencies

All dependencies for phnx are described in cmake and package files.
This means that if you are missing a package somewhere, it will error for you.

To ensure you have all source nodes, be sure to `vcs import` `pheonix.repos` in your workspace `/src`.
Make sure to then Rosdep all these source nodes to drag in all binary dependencies.

If using Webots, you must install webots separately.

## Building

1. Install ROS2 Humble, and ROS2 tools
2. Install Webots
3. Create your ros workspace, a dir of `ws_name/src`
4. In `src`, clone this repo with Git
5. Still in `src`, run `cat Phoenix/phoenix.repos | vcs import` to import source dependencies
6. Cd to the workspace root, and run `rosdep install --from-paths src --ignore-src -r -y` to install binary dependencies
7. Still in workspace root, run `colcon build` to build the workspace. If using clang in your IDE,
   run `colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`
    1. If using the compile commands, they are often created in build. Move it to the workspace root, and import it into
       your IDE tooling.
8. Make sure your user is a part of the unix `dialout` group. This allows for it to connect to USB devices. In addition,
   make sure you
   have [set the udev rules for the OAK-d camera](https://docs.luxonis.com/en/latest/pages/troubleshooting/#udev-rules-on-linux),
   to allow for it to connect.

The repo should now be built, and launch-able in sim or on the kart.

## Launching

### Sim

#### Just sim

By default, webots will be launched:
`ros2 launch phoenix_gazebo common.launch.py`

### With Go-Kart

This repo is designed to be launched on a headless computer on the go-kart. To do this setup the computer with:

- Two Oak-d-w cameras plugged into fast USB ports
- The SICK LiDAR networked to the computer via a switch
- A router connected to the lidar and computer

Once this is done, the kart half of the stack can be launched with:

`ros2 launch phoenix_robot common.launch.py`

This will spawn the autonomy stack, connecting to sensors and the CAN bus. This boots into teleop mode, so it will
not output outputs of the autonomous stack (although it will be running).

It is often useful to start this process on user log on (via chronjob or some feature of a DE like KDE), to allow for
ROS to launch as soon as the PC boots. A script is provided in this repo under scripts called `start.bash` that can be
used to do this, by just setting it as the autorun executable. Note that if using the desktop autostart, you must have 
a monitor connected to actually start the session. We use a fake monitor dongle to achieve this.

Because this is designed to be headless on the kart, it's intended for this computer to be debugged over Wi-Fi. While this 
could be done with an adhoc network, we've found its best to just connect everything to a router, and connect to that for 
extra range.

By connecting with this network, one can share ROS2 instances, allowing for remote debugging and controls.

To achieve this (and actually move the kart), connect another computer to the router. This connection should be
strong, as a poor connection can actually effect the performance of the kart itself. Once connected, run:

`ros2 launch phoenix_robot utilibot.launch.py`

This will launch all nodes relating to teleop and visualization that would exist in sim, but don't run on the headless
kart. Because this ROS network is shared with the kart, this allows us to debug and control it.

Finally, pressing the teleop toggle switch on the controller will allow the autonomous stack to drive the kart. Toggling
it again will swap back to teleop. This can be used as a sort of soft stop. Rviz can be used to view kart state.

#### Teleop only

If you want to just test the kart in a minimal teleop setup, you can use:

`ros2 launch phoenix_robot teleop_only_kartside.launch.py`

This file should be launched on a computer connected to the kart, similar to above. This can be autobooted, or just
a laptop connected to the kart that will also run the utilitbot launch. 

The file above needs to have both the interface board and both cameras connected. These are just used for their IMU.
If desired, you can just use the encoder alone by adding `encoder_only`.