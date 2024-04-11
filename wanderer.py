from random import random
import math
from numpy import inf

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from pymavlink import mavutil
import time

INIT_TIME = 0.5
ARM_TIME = 0.5
TAKEOFF_TIME = 5
TAKEOFF_HEIGHT = 2.5
WALL_DIST = 1.4 # acceptable distance from wall
LAND_TIME = 3

class Subscriber(Node):
    def __init__(self):
        super().__init__('subscriber')
        # susbcribe to the LaserScan ros2 topic
        self.subscription = self.create_subscription(LaserScan, 'scan', self.listener_callback, 10) 
        self.subscription
        self.range_list = [1000.0, 1000.0, 1000.0, 1000.0, 1000.0]
        self.forward_dist = 1000.0
        self.left_forward_dist = 1000.0
        self.left_dist = 1000.0
        self.back_dist = 1000.0
        self.right_dist = 1000.0
        self.right_forward_dist = 1000.0

    def listener_callback(self, msg):
        # map range values to driection of lidar
        # 360 deg lidar for len(msg.ranges[]) = 640
        # ex: robot's 180 deg range value = 320 (180/360 == 320/640)
        self.forward_dist = msg.ranges[0] # 0 deg
        self.left_forward_dist = msg.ranges[80] # 45 deg
        self.left_dist = msg.ranges[160] # 90 deg
        self.back_dist = msg.ranges[320] # 180 deg
        self.right_dist = msg.ranges[480] # 270 deg
        self.right_forward_dist = msg.ranges[560] # 315 deg
        # array to easily grab 5 important ranges 
        self.range_list = [self.left_dist, self.left_forward_dist, self.forward_dist, self.right_forward_dist, self.right_dist, self.back_dist]
        
class CopterState(object):
    stable = 'STABILIZE' # state 0
    guided = 'GUIDED' # state 4
    land = 'LANDED' # state 9

class Copter(object):
    def __init__(self):
        self.vehicle = None
        self.state = CopterState.stable
        self.stop_wander = False

    def initialize(self) -> None:
        '''
        Initialize Function: Set up the Copter for guided flight (phase 1)
        '''
        self.vehicle = mavutil.mavlink_connection('udpin:localhost:14551') # connect to localhost 14551
        self.vehicle.wait_heartbeat()
        # set mode to guided
        if self.state != CopterState.guided:
            self.vehicle.mav.command_long_send(self.vehicle.target_system, self.vehicle.target_component, 176, 0, 1, 4, 0, 0, 0, 0, 0)
            time.sleep(0.25)
            self.vehicle.mav.command_long_send(self.vehicle.target_system, self.vehicle.target_component, 176, 0, 1, 4, 0, 0, 0, 0, 0)
            
            self.state = CopterState.guided
            print("__initialized__")
            time.sleep(INIT_TIME)
            
    def arm(self) -> None:
        '''
        Arm Function: Arm the Copter for takeoff (phase 2)
        '''
        self.vehicle.mav.command_long_send(self.vehicle.target_system, self.vehicle.target_component, 
                                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0) # arm copter
        
        print("__armed__")
        time.sleep(ARM_TIME)

    def takeoff(self, pos_z: float) -> None:
        '''
        Takeoff Function: Set the Copter to takeoff to a desired height (phase 3)
        Args: 
            pos_z (float): desired height
        '''
        self.vehicle.mav.command_long_send(self.vehicle.target_system, self.vehicle.target_component, 
                                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, pos_z)
        
        print("__taking off__")
        time.sleep(TAKEOFF_TIME)

    def land(self) -> None:
        '''
        Land Fuction: Land the copter from where it hovers 
        '''
        if self.state == CopterState.guided:
            print("__landing__")
            self.vehicle.mav.command_long_send(self.vehicle.target_system, self.vehicle.target_component, 176, 0, 1, 9, 0, 0, 0, 0, 0)

            self.state == CopterState.land

    def velocity(self, vel_x: float, vel_y: float, vel_z: float) -> None:
        '''
        Velocity Function: set copter's target velocity
            Positive values denotate forward x, right y, down z
        Args: 
            vel_x (float), vel_y (float), vel_z (float) | (x,y,z) vector velocities 
        '''
        # print("velocity {:0.1f} {:0.1f} {:0.1f}".format(vel_x, vel_y, vel_z))

        self.vehicle.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, self.vehicle.target_system, self.vehicle.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, int(0b10111000111), 0, 0, 0, vel_x, vel_y, vel_z, 0, 0, 0, 0, 0))

    def yaw(self, heading: float, rel: bool, clockwise: int) -> None:
        '''
        Yaw Function: Change heading of the drone relative to itself or absolute to a compass
        Args: 
            heading (float): degs to yaw (relative) or face (absolute)
            rel (bool): yaw based on relative heading(1) or absolute heading(0)
            clockwise (int): yaw clockwise(1) or counter clockwise(-1)
        '''
        # call velocity command
        if clockwise == 1:
            print("Yaw {:0.1f} degrees clock-wise".format(heading))
        else:
            print("Yaw {:0.1f} degrees counter-clock-wise".format(heading))
        self.vehicle.mav.command_long_send(self.vehicle.target_system, self.vehicle.target_component, 
                                     mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0, heading, 45, clockwise, rel, 0, 0, 0)
    
    def account_for_inf(self, subscriber: Subscriber) -> list[(float)]:
        '''
        Account For Infinite: Change range_list[] values to 10.0 if they are infinite
            List is forrmatted [left, left_forward, forward, right_forward, right]
        Args: 
            subscriber (Subscriber): all of the Cartographer LaserScan parameters
        Return: 
            usable_range_list (Array[(float)]): local list replacing inf -> 10.0
        '''
        rclpy.spin_once(subscriber)

        usable_range_list = []
        for range_val in subscriber.range_list:
            if range_val == inf:
                range_val = 10.0
            usable_range_list.append(range_val)

        return usable_range_list

    def display_ranges(self, usable_range_list: list[(float)]) -> None:
        '''
        Display Ranges: Output an instance of the account_for_inf() list in a formatted string
        Args:
            usable_range_list (list[(float)]): local list replacing inf -> 10.0
        '''
        print("`````````````{0:.2f}``````````````\n`````{1:.2f}```````````{2:.2f}``````\n```````````````````````````````\n```````````````````````````````\n```````````````````````````````\n`{3:.2f}`````````C``````````{4:.2f}\n```````````````````````````````\n```````````````````````````````\n```````````````````````````````\n`````````````{5:.2f}``````````````\n".format(usable_range_list[2], usable_range_list[1], usable_range_list[3], usable_range_list[0], usable_range_list[4], usable_range_list[5]))

    def check_lidar(self, subscriber: Subscriber) -> bool:
        '''
        Check Lidar: Check forward facing LiDAR distances so no walls are struck
        Args:
            subscriber (Subscriber): all of the Cartographer LaserScan parameters
        Return:
            False if any of the forward LiDAR ranges are too close, True otherwise
        '''
        usable_range_list = self.account_for_inf(subscriber)
        check = [ range > WALL_DIST for range in usable_range_list[1:4] ] # loop through 3 front-facing range values to check if they are too close to wall
        for bool in check:
            if bool == False: # one of the ranges is too close, stop moving forward
                return False
        return True # all 3 ranges pass
    
    def yaw_to_opening(self, subscriber: Subscriber) -> None:
        '''
        Yaw To Opening: yaw in the direction with the least required rotation
        Args:
            subscriber (Subscriber): all of the Cartographer LaserScan parameters
        '''
        print("yaw_to_open function")
        usable_range_list = self.account_for_inf(subscriber)
        self.display_ranges(usable_range_list) # display the ranges in string format

        # when turning left
        if usable_range_list[1] > usable_range_list[3]: # left_forward > right_forward range
            # calculate distance to ensure robot wont move into the same wall
            req_dist_right = math.sqrt(2 * usable_range_list[4]) 
            # stay in turn loop until tan(theta) of right_forward_dist and right_dist > 45 deg and forward_dist > WALL_DIST
            while subscriber.right_forward_dist < req_dist_right or subscriber.forward_dist < WALL_DIST:
                # smaller yaw angle = exit angle more parallel to wall
                self.yaw(15 + (random()*5), 1, -1) # left turn 10-20 degs
                time.sleep(1.4) # sleep to allow full rotation

                usable_range_list = self.account_for_inf(subscriber) # post-yaw ranges
                self.display_ranges(usable_range_list)
                req_dist_right = math.sqrt(2 * usable_range_list[4])
        else: # when turning right
            req_dist_left = math.sqrt((2 * math.sqrt(usable_range_list[0])))
            while subscriber.left_forward_dist < req_dist_left or subscriber.forward_dist < WALL_DIST:
                self.yaw(15 + random()*5, 1, 1) # right turn 10-20 degs
                time.sleep(1.4)

                usable_range_list = self.account_for_inf(subscriber)
                self.display_ranges(usable_range_list)
                req_dist_left = math.sqrt(2 * usable_range_list[0])
        
    def forward_until_obstacle(self, subscriber: Subscriber) -> None:
        '''
        Forward Until Obstacle: call continuous forward velocity until wall is reached
        Args:
            subscriber (Subscriber): all of the Cartographer LaserScan parameters
        '''
        print("forward_until_obstacle function")
        while self.check_lidar(subscriber): # move until wall
            self.velocity(-1.0, 0.0, 0.0)
            usable_range_list = self.account_for_inf(subscriber)
            self.display_ranges(usable_range_list)
        self.velocity(0.0, 0.0, 0.0) # too close to wall, stop moving

    def wander_maze(self, subscriber: Subscriber) -> None:
        '''
        Wander Maze: wander through the maze while avoiding obstacles
        Args:
            subscriber (Subscriber): all of the Cartographer LaserScan parameters
        '''
        print("__running script__")
        try:
            while not self.stop_wander:
                self.forward_until_obstacle(subscriber)
                self.yaw_to_opening(subscriber)
        except KeyboardInterrupt: # land copter when program is ended
            print("__COMMAND RECEIVED: LANDING COPTER__")
            self.stop_wander = True
            self.land()
        
def main(args=None):
    rclpy.init(args=args)
    subscriber = Subscriber()
    try:
        my_copter = Copter()
        my_copter.initialize()
        my_copter.arm()
        my_copter.takeoff(TAKEOFF_HEIGHT)

        if not my_copter.stop_wander:
            my_copter.wander_maze(subscriber)
    except KeyboardInterrupt: # abort launch
        print("__COMMAND RECEIVED: ABANDONING LAUNCH__")
        my_copter.stop_wander = True
        my_copter.land()

    time.sleep(LAND_TIME)
    print("__exiting script__")
    subscriber.destroy_node

if __name__ == '__main__':
    main()
    