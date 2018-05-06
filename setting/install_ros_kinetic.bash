#!/bin/bash

echo "Install ROS Kinetic"
read -r -p "Are you sure? [y/N] " response
if [[ "$response" =~ ^([yY][eE][sS]|[yY])+$ ]]
then
    echo "======== Install ROS Kinetic. ========";
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list';
    sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116;
    sudo apt-get update;
    sudo apt-get install ros-kinetic-desktop-full;
    sudo rosdep init;
    rosdep update;
    sudo apt-get install python-rosinstall
    sudo apt-get install ros-kinetic-joy
    sudo apt-get install ros-kinetic-hector-sensors-description 
    echo "======== Finish Install. ========";

fi	
