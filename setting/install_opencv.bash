#!/bin/sh
echo " Update & Upgrade the package lists ";
echo
sudo apt-get update;
sudo apt-get upgrade;
echo " Install Git "
echo
sudo apt-get install git;
echo " Install compiler such as g++, gcc, make and etc. ";
echo
sudo apt-get install build-essential;	
sudo apt-get install cmake;
sudo apt-get install pkg-config;
echo " Install libraries for read images from disk. ";
echo
sudo apt-get install libjpeg8-dev libtiff5-dev libjasper-dev libpng12-dev;
echo " Install libraries for read videos from disk. ";
echo
sudo apt-get install libavcodec-dev libavformat-dev; 
sudo apt-get install libswscale-dev libv4l-dev;
sudo apt-get install libxvidcore-dev libx264-dev;
echo " Install GTK for use GUI in OpenCV. ";
echo
sudo apt-get install libgtk2.0-dev;
sudo apt-get install libgtk-3-dev;
echo " Install libraries for optimize various functionalities in OpenCV. ";
echo
sudo apt-get install libatlas-base-dev gfortran;
echo " Install Python 2.7 ";
echo
sudo apt-get install python2.7-dev
pip install numpy
echo "<--------------- Install OpencCV 3 with Python 2 --------------->"
echo
sleep 5
cd ~
echo " Git clone OpenCV and OpenCV_Contrib"
echo
mkdir install_opencv
cd install_opencv
git clone https://github.com/opencv/opencv.git;
git clone https://github.com/opencv/opencv_contrib.git;
cd ~/install_opencv/opencv
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE \
	-D CMAKE_INSTALL_PREFIX=/usr/local \
	-D INSTALL_C_EXAMPLES=ON \
	-D INSTALL_PYTHON_EXAMPLES=ON \
	-D OPENCV_EXTRA_MODULES_PATH=~/install_opencv/opencv_contrib/modules \
	-D BUILD_EXAMPLES=ON ..
make -j4
sudo make install
sudo ldconfig
