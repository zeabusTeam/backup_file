cd ~/catkin_ws/src
hg clone https://bitbucket.org/saringkarnp/modbus-ascii-master
cd ~/catkin_ws/src/modbus-ascii-master
hg pull
hg update beta
cd ~/catkin_ws/src
git clone http://github.com/freefloating-gazebo/freefloating_gazebo.git
cd ~/catkin_ws/src/freefloating_gazebo
git pull
cd ~/catkin_ws/src
git clone https://github.com/anqixu/ueye_cam.git 
cd ~/catkin_ws/src/ueye_cam
git pull

url='https://bitbucket.org/hiveground/'
packages=('hg_ros_teledyne_dvl'       
'hg_ros_3dm_gx4'       
'hg_ros_trax'               
'hg_ros_kvh_imu'       
'hg_ros_tritech_altimeter'  
'hg_ros_pololu'       
'hg_ros_3dm_gx4')

for package in "${packages[@]}"
do
  hg clone $url$package             
  cd ~/catkin_ws/src/$package
  hg pull
  cd ~/catkin_ws/src
done

url='https://bitbucket.org/zeabus2015/'
packages=('zeabus'
'zeabus_ui'
'zeabus_vision')
for package in "${packages[@]}"
do
  hg clone $url$package             
  cd ~/catkin_ws/src/$package
  hg pull
  if [ $package = 'zeabus' ]; then
    hg update zeabus
  else
    hg update
  fi
  cd ~/catkin_ws/src
done

cd ~/catkin_ws/
catkin_make
retval=$?
while [ $retval -ne 0 ]; do
  catkin_make
  retval=$?
done
git clone git://github.com/amix/vimrc.git ~/.vim_runtime
sh ~/.vim_runtime/install_basic_vimrc.sh
echo 'finish the install procedure, please double check if possible'
