# ardupilot-ros-navigation-scripts
Navigation Scripts to Maneuver an Ardupilot Copter, and map, through a Virtual Maze Enviornment using built in Odometry, GPS, and an integrated LiDAR.

Acronyms:
SITL: Software In The Loop
SLAM: Simultaneous Localization and Mapping
LiDAR: Light Detection And Ranging

Project Description

This Project’s goal is to build a deep understanding of the necessary environments, software packages, and communication networks to introduce myself to the field of autonomous robot navigation using LiDAR.  To do this, I will build off from the following open source tools to set up a SITL 3D simulation of an Ardupilot Copter capable of SLAM using:

Ardupilot SITL for the simulated 3D copter 
Gazebo Garden for the simulation and 3D maze environment
Google Cartographer to generate SLAM using a 360 degree 2D LiDAR
ROS2 Humble as the robot communication network
Mavlink as the Flight Controller
PyMavlink to communicate between the ROS2 topics and the drone’s base_link 

This project’s workspace branches from the ardupilot_ros pull request which integrates ROS2’s Nav2 feature.  This project’s scope involves the creation of python two scripts which use the same sensor data to maneuver the drone in a 3D maze using different navigation logic.

The two scripts are:
- Wanderer.py: copter moves forward and maps the environment, yawing when it gets too close to a wall
- Wall_follower: copter maneuvers through the maze environment using the left/right hand rule

Project Architecture

What I’ve learned
ROS2 robotics software library
The ROS2 software package NAV2, learned through online course
Dual booting my computer, ubuntu terminal architecture
The hardware and software used in an automated drone, and their communication protocols
Basic drone avionics

Downside of current project and what to work on next
next: Purchase a pixhawk4 & RPLidarA2. In order to integrate Ros's Nav2 feature with an ardupilot robot, a physical companion computer is required for the following reason. Nav2 requires that the robot be localized from cartographer's information before it can function. In order to accomplish this, the flight controller needs to be transitioned from GPS to non-GPS navigation by changing the EKF source parameters to ExternalNAV. Ardupilot's EKF (Extended Kalman Filter) is responsible for estimating vehicle position. Cartographer's data is only deemed reliable by the EKF filter if it's sent through a Data Distribution Service from the companion computer hardware, which itself gets its odometry data from physical devices like lidar.

Understanding why Nav2 wasn't working by unwinding this knot of requirements took longer than I'd care to admit. However, using computational thinking by teaching myself about
1: the Nav2 requirements (Navigation, Localization, Feedback)
2: Localizing robots (using GPS/non-GPS hardware and software)
3: Editing Ardupilot's EKF parameters (via mavlink)
4: DDS middleware protocol & the ROS2 environment
5: the role of a robot's companion computer
gave me a far greater understanding of robotics and autonomous navigation, which I would have ever discovered if I had mindlessly followed a tutorial.

While much of the source code is capable of being run in a virtual environment, Ardupilot's Extended Kalman Filter unfortunately needs a companion computer to switch

This hardware is needed to create a companion computer which will handle the topics which utilize DDS-XRCE communication.
