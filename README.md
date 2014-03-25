thermo_cam
==========

A repository for the thermal_cam which provices ros nodes to fuse 2D thermal images and 3d pointclouds.

## Fusion of 2D thermal images and 3D pointclouds (based on ROS Fuerte)
### Summary
Volksbot developed by Fraunhofer IAIS Germany.
- Maxon motor system 
- Dynamixel based gripper system
- 3D Scanner based on a LMS1xx sensor and a rotation unit
- Ackermann steering system
![VOLKSBOT"](http://s1.directupload.net/images/131219/4v3rnfu5.jpg "VOLKSBOT")

### Installation and usage for 3D Scanner
- Installation of neccesary packages

> rosdep install 3DLSKdriver (For Eigen3 and PCL)

> rosmake 3DLSKdriver

- Connect the laser scanner to your computer. Both devices, your comptuer and the scanner device should be in the same network.
  - Computer: 192.168.0.x (netmask 255.255.255.0) 
  - Scanner: 192.168.0.12
  - ifconfig eth9 192.168.0.12 netmask 255.255.255.0 up
- The rotation device is connected via USB. You have to set neccessary device permissions
  - sudo chmod 777 /dev/ttyUSB*
- The IP of the laser scanner has to be configured in the following launch File: lms1xx_3dls_x.launch 

> <param name="host" value="192.168.0.102" />

- The rotational velocity and the number of ticker per round can be set in rotary_3dls_x.launch

> <param name="rpm" value="144" />

> <param name="ticks" value="105838" />

- Start up the scanning system
> roslaunch 3DLSKdriver scanmode_multi.launch

- Use Rviz to see the created pointclouds
> rosrun rviz rviz -d rviz_config.rviz

- There are a couple of launch files in the package
  - lms1xx.launch to start the scanner unit 
  - rotary_3dls... to start the rotation unit
  - scan2cloud... to created a pointcloud from several scans
  - cloud2pcd... to convert a cloud to a pcd file
