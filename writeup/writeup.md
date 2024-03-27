# PROJECT WRITE UP

### Acronyms:
- **SITL**: Software In The Loop
- **SLAM**: Simultaneous Localization and Mapping
- **LiDAR**: Light Detection And Ranging
- **GCS**: Ground Control Station

## Goals 
This Project’s goal for me as a software engineer, is to build a deep understanding of the environments, software packages, and communication networks necessary to introduce myself to the field of autonomous robot navigation using LiDAR.  

In preparation of this project, I have completed up to the advanced section of the ROS 2 Humble [Tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate.html), [This](https://www.udemy.com/course/ros2-nav2-stack/learn/lecture/35699436#overview) Udemy course covering the Nav 2 stack, and watched some of Intelligent Quad’s YouTube [videos](https://www.youtube.com/@IntelligentQuads) relevant to my project.

## Description
As for the environment, I will build off from the following open source tools to set up a 3D simulation of a SITL Ardupilot Copter capable of SLAM using:

- Ardupilot SITL for the simulated 3D copter
- Gazebo Garden for the simulation and 3D maze environment
- Google Cartographer to generate SLAM using a 360 degree 2D LiDAR
- ROS 2 Humble as the robot communication network
- A virtual MatekF405-Wing as the Flight Controller, integrated with the GCS MavProxy (Any board capable of SITL should work)
- PyMavlink to communicate between the ROS 2 topics and the drone’s base_link in my scripts

This project’s workspace branches from the ardupilot_ros pull request which integrates ROS 2’s Nav2 feature.  This project’s scope involves the creation of two Python scripts which use the same ROS 2 sensor data to maneuver the drone in a 3D maze using different navigation logic.

The two scripts are:
- [Wanderer](./wanderer.py): copter moves forward and maps the environment, yawing when it gets too close to a wall
- [Wall_Follower](./wall_follower.py): copter maneuvers through the maze environment using the left/right hand rule

## Project Architecture
The scripts showcase the numerous ways to navigate a drone using a LiDAR and GPS.  This allows the flight controller to receive intelligent navigation commands without a companion computer, which costs hundreds of dollars.  
The architecture starts with a SITL ArduCopter integrated with a LiDAR in a ROS 2 Repository. The Copter is also integrated with Cartographer, allowing the LidAR information to map its whereabouts. Using a virtual MatekB405-Wing as the flight controller, 

A basic diagram of the how the 2 scripts interact with Ardupilot’s Communication Protocol [here](Robot_Architecture.png)

### Visualising the Nav Process in the First Pass: [wanderer.py](../wanderer.py)

1. Once Connection is established and the Drone is hovering, call ```pos_script()```

<div align="center">
  <img src="https://github.com/McGovern7/ardupilot-nav-scripts/assets/98053643/d8c5b569-250f-46ba-accb-059936c3ef0e"
    width="600"
    height="290"/>
</div>

2. Unless the program is quit, call ```forward_until_obstacle()```
<div align="center">
  <img src="https://github.com/McGovern7/ardupilot-nav-scripts/assets/98053643/0a377d07-0401-4f23-ad57-620daa47844f"
    width="600"
    height="200"/>
</div>

3. While ```check_lidar()``` remains true, move forward indefinitely. Once ```check_lidar() = false``` (front LiDAR ranges too close), set ```velocity = 0``` and exit function, calling ```yaw_to_open()```
<div align="center">
  <img src="https://github.com/McGovern7/ardupilot-nav-scripts/assets/98053643/6c58fcbf-8bb0-4d86-89e9-278bd1e04a13"
    width="900"
    height="500"/>
</div>

4. Yaw in the direction closest to an opening, ensuring the copter does not pitch straight back into the same wall with some trig calculations.
5. Exit function and return back to the while loop in ```pos_script()```, where once again ```forward_until_obstacle()``` and ```yaw_to_opening()``` are called in sequence until the user exits the program.

### Visualising the Nav Process: [wall_follower.py](../wall_follower.py)

1. Once Connection is established, prompt the user to navigate the maze using the *right-hand rule* or *left-hand rule*. The program then yaws to hug the wall from the correct side, and begins navigation by calling ```escape_maze()```
2. Unless the program is quit, call ```forward_until_snag()```
3. Pitch forward indefinitely with ```velocity = 1``` until the copter is either obstructed by a forward wall, or no longer hugging the wall designated by the user to follow. Either case stops the copter and exits the function, where the script then calls ```yaw_to_opening()```
4. A truth table now determines the required movements to clear the three scenarios

### Demonstration

### What I’ve learned
- ROS 2 robotics software library
- The ROS 2 software package NAV2, learned through online course
- Dual booting my computer, ubuntu terminal architecture
- The hardware and software used in an automated drone, and their communication protocols
- Basic drone avionics
- The importance of Computation Thinking to solve seemingly insurmountable problems piece by piece
- Contributing to an open source software library 

### Downside of current project and what to work on next
Next: Purchase a pixhawk4 & RPLidarA2.

In order to totally integrate ROS 2's Nav2 feature with an Ardupilot robot, a physical companion computer is required for the following reason. Nav2 requires that the robot be localized from Cartographer's information before it can function. In order to accomplish this, Cartographer's information needs to be written into Ardupilot's EKF Filter.  This is done by transitioning the Flight Controller's source parameters from GPS to ExternalNAV. Ardupilot's EKF (Extended Kalman Filter) is responsible for estimating vehicle position. Cartographer's data is only deemed reliable by the EKF filter if it's sent through a Data Distribution Service from the companion computer hardware, which itself gets its odometry data from physical devices like lidar.

Understanding why Nav2 wasn't working by unwinding this knot of requirements took longer than I'd care to admit. However, using computational thinking by teaching myself about

1. the Nav2 requirements (Navigation, Localization, Feedback)
2. Localizing robots (using GPS/non-GPS hardware and software)
3. Editing Ardupilot's EKF parameters (via MAVProxy)
4. DDS middleware protocol & the ROS 2 environment
5. The role of a robot's companion computer gave me an understanding of robotics systems and autonomous navigation far greater than I would have gotten if I had mindlessly followed a tutorial

