
# eDrone

Mission API/SDK for Drone Application SW based on PX4/ROS



## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.



### Prerequisites

The following SW should be preinstalled to develop application SW using eDrone API/SDKs 

- Linux (Ubuntu) (https://www.ubuntu.com/download)
  
  - Ubuntu 16.04.03 LTS is recommended  

- ROS (http://www.ros.org/)

  - ROS Kinetic Kame is recommended

- MAVROS (https://github.com/mavlink/mavros)

  - MAVLink to ROS gateway library

You also need to install the following SW (on the same PC or on a remote one) for a simulation test

- PX4 Firmware (https://github.com/PX4/Firmware)

- Gazebo (http://gazebosim.org/)

  - Version 7.x is recommended (check combination of ROS/Gazebo - http://gazebosim.org/tutorials/?tut=ros_wrapper_versions)

- qGroundControl (https://github.com/mavlink/qgroundcontrol)

  - Version 3.2.x is recommended (curretly the latest release)

 % please refer to (https://dev.px4.io/en/simulation/) for the detailed explanation for SITL and HITL




#### Installing


please refer to gitbook "https://sooyoungmoon1.gitbooks.io/edrone-developers-manual/content/" for details!

1) If you have installed ROS, there should be a workspace directory (ex. catkin_ws)

   Change the current directory to the ROS workspace directory in terminal window

  
2) "Clone" the eDrone SDK code by typing the following command

   $ git clone https://github.com/sooyoungmoon/eDrone


3) Build the code by following the steps:

 (1) Build the 'eDrone_lib' package

     $ catkin build eDrone_lib


 (2) Build the 'examples' package

     $ catkin build examples


##### Test


1) PX4 (SITL) build and execution  

  (1) Move to the PX4 firmware root directory (ex. ~/src/Firmware)

	  $ cd ~/src/Firmware

  (2) Use 'make' command to build & execute PX4 firmware in SITL mode

	# if you use jmavsim simulator
	$ make px4_sitl_default jmavsim
	
	# or, you can use the gazebo simulator
	$ make px4_sitl_default gazebo
 
2) qGroundControl execution

   (AppImage)

   $ chmod +x ./QGroundControl.AppImage
   $ ./QGroundControl.AppImage  (or double click)

   (Compressed Archive)

   $ tar jxf QGroundControl.tar.bz2
   $ cd qgroundcontrol
   $ ./qgroundcontrol-start.sh

     	
3) Execution of the tol example

  (1) Edit launch file 

	런치 파일 내용 중 PX4 주소를 시스템 구성에 맞게 수정

	Edit the launch file according to your system configuration (the actual address of the PX4 SITL)
  	

	<ROS workspace>\src\mission_lib\launch\mission_lib.launch 

	
	<arg name="fcu_url" default="udp://:14540@129.254.189.194:14557" />

	=> <arg name="fcu_url" default="udp://:14540@<IP Addr OF PX4 SITL:14557" />

  (2) Move to the examples package directory

  $ cd ~/catkin_ws/src/examples


4) Execute the tol (takeoff and landin) example

  (Takeoff)

  $ ./tol.sh

5) Check the execution result

   UAV position/attitude/Velocity: can be shown by Gazebo & qgroundControl

   the execution result of an application example: can be monitored through the terminal window

   
####### License

This project is licensed under GPLv3 License - see the [LICENSE.md](LICENSE.md) file for details

######## Acknowledgments



