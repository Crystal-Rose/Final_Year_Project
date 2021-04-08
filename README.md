# Final_Year_Project

This repo must be used with the interbotix repository: https://github.com/Interbotix/interbotix_ros_manipulators/tree/main/interbotix_ros_xsarms
The yaml files provided in this repository must be updated in the Interbotix_ws to work.

This repo uses the reactorX 150 arm from Trossen Robotics, ensure it is connected before executing files.

To get started, open up a terminal and type:

```roslaunch interbotix_xsarm_control xsarm_control.launch robot_model:=rx150```

In another terminal, navigate to the directory get_load.py is in and enter:

```python get_load.py        # python3 bartender.py if using ROS Noetic```
