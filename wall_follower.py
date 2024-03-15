from numpy import inf

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from pymavlink import mavutil
import time

ARM_TIME = 0.5
TAKEOFF_TIME = 5
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
        self.heading = 0

    '''
    Initialize Function: Set up the Copter for guided flight (phase 1)
    '''
    def initialize(self):
        self.vehicle = mavutil.mavlink_connection('udpin:localhost:14551') # connect to localhost 14551
        self.vehicle.wait_heartbeat()
        # set mode to guided
        if self.state != CopterState.guided:
            self.vehicle.mav.command_long_send(self.vehicle.target_system, self.vehicle.target_component, 176, 0, 1, 4, 0, 0, 0, 0, 0)
            time.sleep(0.25)
            self.vehicle.mav.command_long_send(self.vehicle.target_system, self.vehicle.target_component, 176, 0, 1, 4, 0, 0, 0, 0, 0)
            
            self.state = CopterState.guided
            print("__initialized__")
            time.sleep(0.5)
            self.arm() # move to next phase of launch

    '''
    Arm Function: Arm the Copter for takeoff (phase 2)
    '''
    def arm(self):
        self.vehicle.mav.command_long_send(self.vehicle.target_system, self.vehicle.target_component, 
                                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0) # arm copter
        
        print("__armed__")
        time.sleep(ARM_TIME)
        self.takeoff(2.5) # move to next phase of launch

    '''
    Takeoff Function: Set the Copter to takeoff to a desired height (phase 3)
    Params: pos_z | desired height
    '''
    def takeoff(self, pos_z):
        self.vehicle.mav.command_long_send(self.vehicle.target_system, self.vehicle.target_component, 
                                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, pos_z)
        
        print("__taking off__")
        time.sleep(TAKEOFF_TIME)

    '''
    Land Fuction: Land the copter where it hovers 
    '''
    def land(self):
        if self.state == CopterState.guided:
            print("__landing__")
            self.vehicle.mav.command_long_send(self.vehicle.target_system, self.vehicle.target_component, 176, 0, 1, 9, 0, 0, 0, 0, 0)

            self.state == CopterState.land

    """
    Position Function: set copter's target position, exit function when waypoint distance has been met
    Params: pos_x, pos_y, pos_z | (x,y,z) coordinates of desired location
    """
    def position(self, pos_x, pos_y, pos_z):
        print("To Position: ({:0.1f},{:0.1f},{:0.1f})".format(pos_x, pos_y, pos_z))

        self.vehicle.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, self.vehicle.target_system, self.vehicle.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, int(0b110111111000), pos_x, pos_y, pos_z, 0, 0, 0, 0, 0, 0, 0, 0))

        # check NAV_CONTROLLER_OUTPUT to determine when the waypoint distance has been truly reached
        unreached = True
        group_list = []
        while unreached: # loop through wp_dist message until checkpoint confirmed reached by 15 wp_dist = 0s in group_list
            msg = self.vehicle.recv_match(
                type='NAV_CONTROLLER_OUTPUT', blocking=True)
            
            group_list.append(msg.wp_dist)
            if len(group_list) == 10:
                if sum(group_list) == 0:
                    print("__waypoint reached__")
                    unreached = False
                else:
                    group_list.clear()
        time.sleep(0.5)

    '''
    Velocity Function: set copter's target velocity
    Params: vel_x, vel_y, vel_z | positive = forward x, right y, down z
    '''
    def velocity(self, vel_x, vel_y, vel_z):
        #print("velocity {:0.1f} {:0.1f} {:0.1f}".format(vel_x, vel_y, vel_z))

        self.vehicle.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, self.vehicle.target_system, self.vehicle.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, int(0b10111000111), 0, 0, 0, vel_x, vel_y, vel_z, 0, 0, 0, 0, 0))

    '''
    Yaw Function: Change heading of the drone relative to itself or absolute to a compass
    heading: degrees to turn (relative) or face (absolute)
    rel: turn based on relative heading or absolute heading
    clockwise: bool turn clockwise or counter clockwise based on current heading
    '''
    def yaw(self, heading, rel, clockwise):
        # turn in direction relative to a drone, or absolute to a compass
        if rel:
            isRel = 1
        else:
            isRel = 0

        # call command
        if clockwise == 1:
            print("Yaw {:0.1f} degrees clock-wise".format(heading))
        else:
            print("Yaw {:0.1f} degrees counter-clock-wise".format(heading))
        self.vehicle.mav.command_long_send(self.vehicle.target_system, self.vehicle.target_component, 
                                     mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0, heading, 45, clockwise, isRel, 0, 0, 0)

    def print_ranges(self, subscriber):
        usable_range_list = self.account_for_inf(subscriber)
        # print range values formatted to approximate direction in unit circle
        print("`````````````{:.2f}``````````````".format(usable_range_list[2]))
        print("```{:.2f}``````````````{:.2f}`````".format(usable_range_list[1], usable_range_list[3]))
        print("```````````````````````````````")
        print("```````````````````````````````")
        print("`{0:.2f}`````````C``````````{1:.2f}".format(usable_range_list[0], usable_range_list[4]))
        print("```````````````````````````````")
        print("```````````````````````````````")
        print("```````````````````````````````")
        print("`````````````{:.2f}``````````````".format(usable_range_list[5]))
        
    '''
    change important range values to 10.0 if they are infinite
    list is forrmatted [left, left_forward, forward, right_forward, right]
    params: Subscriber - all of the Cartographer LaserScan parameters
    '''
    def account_for_inf(self, subscriber):
        rclpy.spin_once(subscriber)

        usable_range_list = []
        for range_val in subscriber.range_list:
            if range_val == inf:
                range_val = 10.0
            usable_range_list.append(range_val)
        return usable_range_list # ranges without any inf values
    
    '''
    call continuous forward position until wall is reached 
    params: Subscriber - all of the Cartographer LaserScan parameters
    '''
    def forward_until_snag(self, subscriber):
        print("forward_until_snag function")
        usable_range_list = self.account_for_inf(subscriber)
        self.print_ranges(subscriber)

        if self.hand_rule: # right-hand
            # move until either forward wall hit or no longer hugging right wall
            while usable_range_list[2] > WALL_DIST and usable_range_list[4] < WALL_DIST:
                self.velocity(-1.0, 0.0, 0.0)
                usable_range_list = self.account_for_inf(subscriber)
                self.print_ranges(subscriber)
        else: # left-hand
            # move until either forward wall hit or no longer hugging left wall
            while usable_range_list[2] > WALL_DIST and usable_range_list[0] < WALL_DIST:
                self.velocity(-1.0, 0.0, 0.0)
                usable_range_list = self.account_for_inf(subscriber)
                self.print_ranges(subscriber)

        self.velocity(0.0, 0.0, 0.0) # too close to wall, stop moving

    '''
    yaw in direction to keep hugging the right wall
    params: Subscriber - all of the Cartographer LaserScan parameters
    '''
    def yaw_to_opening(self, subscriber):
        # truth table: right-hand rule
        # case 1: forward 0 and right 0
        # case 2: forward 1 and right 1
        # case 3: forward 0 and right 1 - unlikely / impossible
        # replace right w/ left for left-hand rule
        print("yaw_to_open function")
        usable_range_list = self.account_for_inf(subscriber)
        self.print_ranges(subscriber)

        if self.hand_rule: # right-hand
            # 0 forward and 0 right -> yaw 90 deg counter_clockwise
            if usable_range_list[2] < WALL_DIST and usable_range_list[4] < WALL_DIST:
                self.yaw(90, 1, -1)
                time.sleep(2.5)
                return
            # 1 forward and 1 right -> manually position around wall edge
            elif usable_range_list[2] > WALL_DIST and usable_range_list[4] > WALL_DIST:
                self.print_ranges(subscriber)
                self.yaw(90, 1, 1) # yaw 90 deg clockwise
                time.sleep(2.5)
                return
            else: # in between positions, turn around
                self.print_ranges(subscriber)
                self.yaw(180, 1, 1)
                time.sleep(4.5)
                return
        else:
            # 0 forward and 0 left -> yaw 90 deg clockwise
            if usable_range_list[2] < WALL_DIST and usable_range_list[0] < WALL_DIST:
                self.yaw(90, 1, 1)
                time.sleep(2.5)
                return
            # 1 forward and 1 left -> manually position around wall edge
            if usable_range_list[2] > WALL_DIST and usable_range_list[0] > WALL_DIST:
                self.print_ranges(subscriber)
                self.yaw(90, 1, -1) # yaw 90 deg counter_clockwise
                time.sleep(2.5)
                return
            else: # in between positions, turn around
                self.print_ranges(subscriber)
                self.yaw(180, 1, 1)
                time.sleep(4.5)
                return
    
    def escape_maze(self, subscriber):
        print("__running script__")
        try:
            while not self.stop_wander:
                self.forward_until_snag(subscriber)
                self.yaw_to_opening(subscriber)
                time.sleep(1.5)
                usable_range_list = self.account_for_inf(subscriber)
                self.print_ranges(subscriber)

                if self.hand_rule:
                    # there is a right wall to the robot's front right, so move a little forward so the right side lidar sees it
                    if usable_range_list[4] > 2.0 and usable_range_list[3] < 2.0:
                        self.print_ranges(subscriber)
                        while not usable_range_list[4] < WALL_DIST:
                            self.velocity(-1.0, 0.0, 0.0)
                            usable_range_list = self.account_for_inf(subscriber)
                            self.print_ranges(subscriber)
                else:
                    # there is a left wall to the robot's front left, so move a little forward so the left side lidar sees it
                    if usable_range_list[0] > 2.0 and usable_range_list[1] < 2.0:
                        self.print_ranges(subscriber)
                        while not usable_range_list[0] < WALL_DIST:
                            self.velocity(-1.0, 0.0, 0.0)
                            usable_range_list = self.account_for_inf(subscriber)
                            self.print_ranges(subscriber)
                # move after yaw to not reenter yaw_to_opening function
        except KeyboardInterrupt: # land copter when program is ended
            self.stop_wander = True
            self.land()
        print("__exiting script__")

def main(args=None):
    rclpy.init(args=args)
    subscriber = Subscriber()

    my_copter = Copter()
    my_copter.initialize()

    my_copter.hand_rule = int(input("Type '0' for left-hand rule, and '1' for right-hand rule: "))
    while my_copter.hand_rule != 0 and my_copter.hand_rule != 1:
        my_copter.hand_rule = int(input("(Input Error) Try Again, Type '0' for left-hand rule, and '1' for right-hand rule: "))
    
    if my_copter.hand_rule == 1: # right-hand rule
        my_copter.yaw(180, 1, 1)
        time.sleep(4.5)
        my_copter.escape_maze(subscriber)
    elif my_copter.hand_rule == 0: # left-hand rule
        my_copter.yaw(90, 1, 1)
        time.sleep(2.5)
        my_copter.escape_maze(subscriber)
    
    time.sleep(LAND_TIME)
    subscriber.destroy_node

if __name__ == '__main__':
    main()
    