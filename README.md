<div align="center">
  <img src="https://github.com/McGovern7/ardupilot-nav-scripts/assets/98053643/818c757b-7715-4b0e-954b-45e25509b9e1" alt="Ardupilot LiDAR Navigation Scripts"/>
</div>
Navigation Scripts to Maneuver an Ardupilot Copter, and map, through a Virtual Maze Environment using built in Odometry, GPS, and an integrated LiDAR.

## DESCRIPTION
This project’s workspace branches from the ardupilot_ros pull request which integrates ROS2’s Nav2 feature.  This project’s scope involves the creation of two Python scripts which use the ROS2 LaserScan data to maneuver the drone in a 3D maze using different navigation logic. 

## INSTALLATION
Recommended Installation Video [Here](https://www.youtube.com/watch?v=2BhyKyzKAbM)

### Software Requirements
- ROS Humble
- Gazebo Garden
- Cartographer ROS

### Workspace Requirements
- ardupilot_gz
- cartographer_ros
- ardupilot_ros PR - "WIP: add integration with nav2"

## BUILD
In order to launch the environment before running the scripts, the following commands need to be launched in **Separate** terminals:

```
# Launch the Gazebo simulation
cd ~/ros2_ws
ros2 launch ardupilot_gz_bringup iris_maze.launch.py rviz:=false
```
```
# Launch Cartographer to generate SLAM
cd ~/ros2_ws
ros2 launch ardupilot_ros cartographer.launch.py rviz:=false
```
```
# Launch Rviz with Nav2 (NavToPose only works if Copter’s been localized with a companion computer)
cd ~/ros2_ws
ros2 launch ardupilot_ros navigation.launch.py
```
```
cd ~/ros2_ws
mavproxy.py --console --map --aircraft test --master=:14550
```
```
cd ~ros2_ws/src/ardupilot_ros/scripts
# run your preferred script after Mavlink connection is initialzied
python3 wall_follower.py
```

Enter ctrl+c in the script terminal to stop navigation and land the copter. 

New navigation scripts can be ran on the same terminal after the copter lands
