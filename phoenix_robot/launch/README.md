Much like phoenix_training, these launch files attempt to use as much as possible from
phoenix_gazebo to help cut down on redundancy.

All files used from PG should make sure to set 'use_sim_time' false.

## Usage

These launch files are split between the files used on the phoenix onboard PC itself,
and the remote PC teleoping.

Because we currently use drive mode switch, you must use teleop alongside auton, else
you will be unable to press the button that swaps from teleop to auton.

General setup:
1. Network the onboard PC and your remote PC together
2. On Phnx, launch the appropriate launch file for your mode of operation
   2. Ex. `ros2 launch phoenix_robot data_collect.launch.py`
3. On the remote PC, `ros2 launch phoenix_robot utilibot.launch.py`
   4. This will assume you are using a wheel. If not,  `ros2 launch phoenix_robot utilibot.launch.py use_wheel:=false` to use a joystick

