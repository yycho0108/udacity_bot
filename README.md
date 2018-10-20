# Udacity\_Bot

Primary ROS Package for Udacity Robotics Nanodegree Term 2 Project 2 : Robot Localization.

Note `scripts/calibrate.py` and `scripts/scan_to_angle.py` are from the `turtlebot_calibration` package.

## Running

```bash
roscore
roslaunch udacity_bot udacity_world.launch robot:=udacity_bot
roslaunch udacity_bot amcl.launch
roslaunch udacity_bot move_base.launch type:=teb
rosrun udacity_bot navigation_goal
```
