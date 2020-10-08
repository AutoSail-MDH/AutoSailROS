# AutoSail-MDH
AutoSail-MDH is a Mälardalens Högskola (MDH) student project whos aim is to produce an autonomous sailing boat to compete in the World Robot Sailing Champtionship (WRSC) & International Robotic Sailing Championship (IRSC).

# AutoSailROS
Our first goal is to create a proof of concept using an RC sailboat installed with Robot Operating System (ROS) Melodic. AutoSailROS repository contains the ROS packages used on the boat.

# Installation
The system is running on _ROS Melodic_ which has only been tested on _Ubuntu 18.04_. Follow the installation guide for ROS Melodic to continue
http://wiki.ros.org/melodic/Installation/Ubuntu

Build a _catkin_ workspace with these instructions http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

When the workspace is setup, go to `<workspace_name>/src` and clone this repository. Go to the root of the workspace and run `catkin_make`
