Itegration of ROS with the OpenROV

install ros...

install rosbridge & gscam
```
sudo apt-get install ros-indigo-rosbridge-server

sudo apt-get install -y ros-indigo-rosbridge-suite gstreamer0.10 libgstreamer-plugins-base0.10-dev ros-indigo-image-transport ros-indigo-camera-calibration-parsers ros-indigo-camera-info-manager

cd ~/catkin_ws/src
git clone https://github.com/ros-drivers/gscam
cd ..
catkin_make
```

install openrov_ros in your catkin workspace
```
cd ~/catkin_src
git clone https://github.com/laughlinbarker/openrov_ros.git openrov
cd ..
catkin_make
```
