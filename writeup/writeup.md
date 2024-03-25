## Project Description

### Acronyms:

- **SITL**: Software In The Loop
- **SLAM**: Simultaneous Localization and Mapping
- **LiDAR**: Light Detection And Ranging

This Project’s goal is to build a deep understanding of the environments, software packages, and communication networks necessary to introduce myself to the field of autonomous robot navigation using LiDAR.  To do this, I will build off from the following open source tools to set up a SITL 3D simulation of an Ardupilot Copter capable of SLAM using:

- Ardupilot SITL for the simulated 3D copter 
- Gazebo Garden for the simulation and 3D maze environment
- Google Cartographer to generate SLAM using a 360 degree 2D LiDAR
- ROS2 Humble as the robot communication network
- Mavlink as the Flight Controller
- PyMavlink to communicate between the ROS2 topics and the drone’s base_link 

This project’s workspace branches from the ardupilot_ros pull request which integrates ROS2’s Nav2 feature.  This project’s scope involves the creation of python two scripts which use the same sensor data to maneuver the drone in a 3D maze using different navigation logic.

The two scripts are:
- [Wanderer](./wanderer.py): copter moves forward and maps the environment, yawing when it gets too close to a wall
- [Wall_Follower](./wall_follower.py): copter maneuvers through the maze environment using the left/right hand rule

## Project Architecture

### Drone Communication Chart

### Video Demonstration

### What I’ve learned
- ROS2 robotics software library
- The ROS2 software package NAV2, learned through online course
- Dual booting my computer, ubuntu terminal architecture
- The hardware and software used in an automated drone, and their communication protocols
- Basic drone avionics
- The importance of Computation Thinking to solve seemingly insurmountable problems piece by piece

### Downside of current project and what to work on next
Next: Purchase a pixhawk4 & RPLidarA2. 

In order to totally integrate Ros's Nav2 feature with an ardupilot robot, a physical companion computer is required for the following reason. Nav2 requires that the robot be localized from Cartographer's information before it can function. In order to accomplish this, Cartographer's information needs to be written into Ardupilot's EKF Filter.  This is done by transitioning the Flight Controller's source parameters from GPS to ExternalNAV. Ardupilot's EKF (Extended Kalman Filter) is responsible for estimating vehicle position. Cartographer's data is only deemed reliable by the EKF filter if it's sent through a Data Distribution Service from the companion computer hardware, which itself gets its odometry data from physical devices like lidar.

Understanding why Nav2 wasn't working by unwinding this knot of requirements took longer than I'd care to admit. However, using computational thinking by teaching myself about
1: the Nav2 requirements (Navigation, Localization, Feedback)
2: Localizing robots (using GPS/non-GPS hardware and software)
3: Editing Ardupilot's EKF parameters (via mavlink)
4: DDS middleware protocol & the ROS2 environment
5: the role of a robot's companion computer
gave me a understanding of robotics systems and autonomous navigation far greater than I would have gotten if I had mindlessly followed a tutorial.
