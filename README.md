# AutoSail-MDH
AutoSail-MDH is a Mälardalens Högskola (MDH) student project whos aim is to produce an autonomous sailing boat to compete in the World Robot Sailing Champtionship (WRSC) & International Robotic Sailing Championship (IRSC).

Our first goal is to create a proof of concept using the RC boat _Dragon Flite 95_. It runs on an Odroid C2 single board computer, installed with Robot Operating System (ROS) Melodic. This repository contains the ROS packages used on the boat.

# Installation
The system is running on _ROS Melodic_ which has only been tested on _Ubuntu 18.04_. Follow the installation guide for ROS Melodic to continue
http://wiki.ros.org/melodic/Installation/Ubuntu

Build a _catkin_ workspace with these instructions http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

When the workspace is setup, go to `<workspace_name>/src` and clone this repository. Go to the root of the workspace and run `catkin_make`
