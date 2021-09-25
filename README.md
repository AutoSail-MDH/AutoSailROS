# AutoSail-MDH
AutoSail-MDH is a Mälardalens Högskola (MDH) student project whos aim is to produce an autonomous sailing boat to compete in the World Robot Sailing Champtionship (WRSC) & International Robotic Sailing Championship (IRSC).

#AutoSailROS_Summer2020
This repository contains the Robot Operating System (ROS) packages used for the project course summer 2020. The RC boat _Dragon Flite 95_ were used with a Odroid C2 as the main computer. Our task was to build a wind sensor with 3d printed parts and a rotary encoder. Additional packages were installed to test the xsens IMU.

# Installation
The system is running on _ROS Melodic_ which has only been tested on _Ubuntu 18.04_. Follow the installation guide for ROS Melodic to continue
http://wiki.ros.org/melodic/Installation/Ubuntu

Build a _catkin_ workspace with these instructions http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

When the workspace is setup, go to `<workspace_name>/src` and clone this repository. Go to the root of the workspace and run `catkin_make`
