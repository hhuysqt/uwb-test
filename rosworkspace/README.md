A simple ROS node mark on rviz
===

Usage:
---
Copy the uwb\_serial floader to your workspace, update CMakelist, and catkin\_make.<br>
There're 2 parameters: /uwb/serial identifies the serial device, and /uwb/nr\_anchor sets the number of anchors.<br>
In rviz, set 'Fix Frame' to 'my\_frame', and 'Add' a 'Marker'.


