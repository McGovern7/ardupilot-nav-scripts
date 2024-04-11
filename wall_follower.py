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
        self.range_list = [1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0]
        self.left_dist = 1000.0
        self.left_forward_dist = 1000.0
        self.forward_dist = 1000.0
        self.right_forward_dist = 1000.0
        self.right_dist = 1000.0
        self.back_dist = 1000.0

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
        # array to easily grab 6 important ranges for navigation
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
        self.hand_rule = None
        self.heading = 90.0

    def initialize(self) -> None:
        '''
        Initialize Function: Connect copter to Mavlink, and set flight state to guided (phase 1)
        '''
        self.vehicle = mavutil.mavlink_connection('udpin:localhost:14551') # connect to localhost 14551
        self.vehicle.wait_heartbeat()
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
                                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        
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
        print("Yaw to heading: {:0.1f}".format(heading))
        # call velocity command
        self.vehicle.mav.command_long_send(self.vehicle.target_system, self.vehicle.target_component, 
                                     mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0, heading, 45, clockwise, rel, 0, 0, 0)

    def compass_heading(self, yaw_num: float) -> float:
        '''
        Compass Heading: Calculate the desired heading absolute to a compass
        Args: 
            yaw_num (float): relative heading change (+num -> clockwise | -num -> counter-clockwise)
        Return: 
            self.heading (float): updated absolute heading, used as param for yaw() function
        '''
        # ensure output remains within 360deg compass range
        if self.heading + yaw_num >= 360.0:
            self.heading += yaw_num - 360.0
        elif self.heading + yaw_num < 0.0:
            self.heading += yaw_num + 360.0
        else:
            self.heading += yaw_num
            
        return self.heading

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

    def forward_until_snag(self, subscriber: Subscriber) -> None:
        '''
        Forward Until Snag: Move forward with a set velocity until stopping when a turn is required
        Args: 
            subscriber (Subscriber): all of the Cartographer LaserScan parameters
        '''
        print("forward_until_snag function")
        usable_range_list = self.account_for_inf(subscriber)

        if self.hand_rule: # right-hand
            # pitch forward until either forward wall hit or no longer hugging right wall
            while usable_range_list[2] > WALL_DIST and usable_range_list[4] < WALL_DIST:
                self.velocity(-1.0, 0.0, 0.0)
                usable_range_list = self.account_for_inf(subscriber)
                self.display_ranges(usable_range_list)
        else: # left-hand
            # pitch forward until either forward wall hit or no longer hugging left wall
            while usable_range_list[2] > WALL_DIST and usable_range_list[0] < WALL_DIST:
                self.velocity(-1.0, 0.0, 0.0)
                usable_range_list = self.account_for_inf(subscriber)
                self.display_ranges(usable_range_list)

        self.velocity(0.0, 0.0, 0.0) # too close to wall, stop moving

    def yaw_to_opening(self, subscriber: Subscriber) -> None:
        '''
        Yaw to Opening: Yaw in direction to keep hugging the desired wall
        Args: 
            subscriber (Subscriber): all of the Cartographer LaserScan parameters
        '''
        # see writeup for truth_table
        # key: {0: wall < WALL_DIST, 1: wall > WALL_DIST}
        # case 1 -> 0 forward dir; 0 hand_rule dir
        # case 2 -> 1 forward dir; 1 hand_rule dir
        # case 3 -> 0 forward dir; 1 hand_rule dir
        print("yaw_to_open function")
        usable_range_list = self.account_for_inf(subscriber)
        self.display_ranges(usable_range_list)

        if self.hand_rule: # right-hand
            # case 1 -> yaw compass_heading + 90 deg counter_clockwise
            if usable_range_list[2] < WALL_DIST and usable_range_list[4] < WALL_DIST:
                self.yaw(self.compass_heading(90.0*-1.0), 0, -1)
                time.sleep(2.5)
                return
            # case 2 -> U-turn around wall edge
            elif usable_range_list[2] > WALL_DIST and usable_range_list[4] > WALL_DIST:
                self.yaw(self.compass_heading(90.0*1.0), 0, 1) # yaw compass_heading + 90 deg clockwise
                time.sleep(2.5)
                return
            else: # case 3 -> yaw compass_heading + 180 clockwise
                self.yaw(self.compass_heading(180.0*1.0), 0, 1)
                time.sleep(4.5)
                return
        else: # left-hand
            # case 1 -> yaw compass_heading + 90 deg clockwise
            if usable_range_list[2] < WALL_DIST and usable_range_list[0] < WALL_DIST:
                self.yaw(self.compass_heading(90.0*1.0), 0,  1)
                time.sleep(2.5)
                return
            # case 2 -> U-turn around wall edge
            if usable_range_list[2] > WALL_DIST and usable_range_list[0] > WALL_DIST:
                self.yaw(self.compass_heading(90.0*-1.0), 0, -1) # yaw compass_heading + 90 deg counter_clockwise
                time.sleep(2.5)
                return
            else: # case 3 -> yaw compass_heading + 180 clockwise
                self.yaw(self.compass_heading(180.0*1.0), 0, 1)
                time.sleep(4.5)
                return
    
    def escape_maze(self, subscriber: Subscriber) -> None:
        '''
        Escape Maze: Navigate maze by continuously pitching and yawing, while hugging the left or right wall
        Args: 
            subscriber (Subscriber): all of the Cartographer LaserScan parameters
        '''
        print("__running script__")
        try:
            while not self.stop_wander:
                self.forward_until_snag(subscriber)
                self.yaw_to_opening(subscriber)
                time.sleep(1.5)
                usable_range_list = self.account_for_inf(subscriber)
                self.display_ranges(usable_range_list)

                # needed to continue case 2 -> U-turn
                if self.hand_rule:
                    # check if right wall is unseen by right lidar, but seen by forward_right
                    if usable_range_list[4] > 2.0 and usable_range_list[3] < 2.0:
                        # pitch forward until right lidar sees this wall 
                        while not usable_range_list[4] < WALL_DIST:
                            self.velocity(-1.0, 0.0, 0.0)
                            usable_range_list = self.account_for_inf(subscriber)
                            self.display_ranges(usable_range_list)
                else:
                    # check if left wall is unseen by left lidar, but seen by forward_left
                    if usable_range_list[0] > 2.0 and usable_range_list[1] < 2.0:
                        # pitch forward until right lidar sees this wall
                        while not usable_range_list[0] < WALL_DIST:
                            self.velocity(-1.0, 0.0, 0.0)
                            usable_range_list = self.account_for_inf(subscriber)
                            self.display_ranges(usable_range_list)

        except KeyboardInterrupt: # land copter when program is manually ended
            self.stop_wander = True
            self.land()
        print("__exiting script__")

def main(args=None):
    rclpy.init(args=args)
    subscriber = Subscriber()

    try:
        my_copter = Copter()
        my_copter.initialize()
        my_copter.arm()
        my_copter.takeoff(TAKEOFF_HEIGHT)

        my_copter.hand_rule = int(input("Type '0' for left-hand rule, and '1' for right-hand rule: "))
        while my_copter.hand_rule != 0 and my_copter.hand_rule != 1:
            my_copter.hand_rule = int(input("(Input Error) Try Again, Type '0' for left-hand rule, and '1' for right-hand rule: "))
        
        if not my_copter.stop_wander:
            if my_copter.hand_rule == 1: # right-hand rule
                my_copter.yaw(270.0, 0, -1)
                my_copter.heading = 270.0
                time.sleep(4.5)
                my_copter.escape_maze(subscriber)
            elif my_copter.hand_rule == 0: # left-hand rule
                my_copter.yaw(180.0, 0, 1)
                my_copter.heading = 180.0            
                time.sleep(2.5)
                my_copter.escape_maze(subscriber)
    except KeyboardInterrupt: # abort launch
        print("__COMMAND RECEIVED: ABANDONING LAUNCH__")
        my_copter.stop_wander = True
        my_copter.land()

    time.sleep(LAND_TIME)
    subscriber.destroy_node

if __name__ == '__main__':
    main()
    