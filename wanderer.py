from random import random
import math
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
        self.range_list = [self.left_dist, self.left_forward_dist, self.forward_dist, self.right_forward_dist, self.right_dist]

        # print range values formatted to approximate direction in unit circle on every topic spin
        print("`````````````{:.2f}``````````````".format(msg.ranges[0]))
        print("`````{0:.2f}```````````{1:.2f}``````".format(msg.ranges[80], msg.ranges[560]))
        print("```````````````````````````````")
        print("```````````````````````````````")
        print("`{0:.2f}`````````C``````````{1:.2f}".format(msg.ranges[160], msg.ranges[480]))
        print("```````````````````````````````")
        print("```````````````````````````````")
        print("```````````````````````````````")
        print("`````````````{:.2f}``````````````".format(msg.ranges[320]))
        
class CopterState(object):
    stable = 'STABILIZE' # state 0
    guided = 'GUIDED' # state 4
    land = 'LANDED' # state 9

class Copter(object):
    def __init__(self):
        self.vehicle = None
        self.state = CopterState.stable
        self.stop_wander = False
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
        # TODO: make sure loop doesn't run endlessly if copter gets stuck.
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
        print("velocity {:0.1f} {:0.1f} {:0.1f}".format(vel_x, vel_y, vel_z))

        self.vehicle.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, self.vehicle.target_system, self.vehicle.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, int(0b10111000111), 0, 0, 0, vel_x, vel_y, vel_z, 0, 0, 0, 0, 0))

    '''
    Acceleration Function: set copter's target acceleration
    Params: acc_x, acc_y, acc_z | positive = forward x, right y, down z
    '''
    def accelerate(self, acc_x, acc_y, acc_z):
        print("acceleration {:0.1f} {:0.1f} {:0.1f}".format(acc_x, acc_y, acc_z))

        self.vehicle.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, self.vehicle.target_system, self.vehicle.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, int(0b10000111111), 0, 0, 0, 0, 0, 0, acc_x, acc_y, acc_z, 0, 0))

    '''
    Yaw Function: Change heading of the drone relative to itself or absolute to a compass
    heading: degrees to turn (relative) or face (absolute)
    rel: turn based on relative heading or absolute heading
    clock_wise: bool turn clockwise or counter clockwise based on current heading
    '''
    def yaw(self, heading, rel, clock_wise):
        # turn in direction relative to a drone, or absolute to a compass
        if rel:
            isRel = 1
        else:
            isRel = 0

        # call command
        if clock_wise == 1:
            print("Yaw {:0.1f} degrees clock-wise".format(heading))
        else:
            print("Yaw {:0.1f} degrees counter-clock-wise".format(heading))
        self.vehicle.mav.command_long_send(self.vehicle.target_system, self.vehicle.target_component, 
                                     mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0, heading, 45, clock_wise, isRel, 0, 0, 0)
    
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
    returns true so long as the 3 forward facing sensor wall distances are not too close
    params: Subscriber - all of the Cartographer LaserScan parameters
    '''
    def check_lidar(self, subscriber):
        usable_range_list = self.account_for_inf(subscriber)
        check = [ range > WALL_DIST for range in usable_range_list[1:4] ] # loop through 3 front-facing range values to check if they are too close to wall
        for bool in check:
            if bool == False: # one of the ranges is too close, stop moving forward
                return False
        return True # all 3 ranges pass
    
    '''
    yaw in the direction with the least required rotation
    params: Subscriber - all of the Cartographer LaserScan parameters
    '''
    def yaw_to_opening(self, subscriber):
        print("yaw_to_open function")
        usable_range_list = self.account_for_inf(subscriber)

        # when turning left
        if usable_range_list[1] > usable_range_list[3]: # left_forward > right_forward range
            # calculate distance to ensure robot wont move into the same wall
            req_dist_right = math.sqrt(2 * usable_range_list[4]) 
            # stay in turn loop until tan(theta) of right_forward_dist and right_dist > 45 deg and forward_dist > WALL_DIST
            while subscriber.right_forward_dist < req_dist_right or subscriber.forward_dist < WALL_DIST:
                usable_range_list = self.account_for_inf(subscriber)
                req_dist_right = math.sqrt(2 * usable_range_list[4])

                print("turning left")
                self.yaw(10 + (random()*4), 1, -1) # left turn 6-14 degs
                time.sleep(1.2) # sleep to allow full rotation
        else: # when turning right
            req_dist_left = math.sqrt((2 * math.sqrt(usable_range_list[0])))
            while subscriber.left_forward_dist < req_dist_left or subscriber.forward_dist < WALL_DIST:
                usable_range_list = self.account_for_inf(subscriber)
                req_dist_left = math.sqrt(2 * usable_range_list[0])

                print("turning right")
                self.yaw(10 + random()*4, 1, 1) # right turn 6-14 degs
                time.sleep(1.2) 
        
    '''
    call continuous forward position until wall is reached, then yaw in direction of further left_distance or right_distance
    params: Subscriber - all of the Cartographer LaserScan parameters
    '''
    def forward_until_obstacle(self, subscriber):
        print("forward_until_obstacle function")
        while self.check_lidar(subscriber): # move until wall
            self.velocity(-1.0, 0.0, 0.0)
        self.velocity(0.0, 0.0, 0.0) # too close to wall, stop moving

    '''
    completes the set maze using a script which knows the layout already
    '''
    def pos_script(self, subscriber):
        print("__running script__")
        try:
            while not self.stop_wander:
                self.forward_until_obstacle(subscriber)
                self.yaw_to_opening(subscriber)
        except KeyboardInterrupt: # land copter when program is ended
            self.stop_wander = True
            self.land()
        
    
        
def main(args=None):
    rclpy.init(args=args)
    subscriber = Subscriber()

    myCopter = Copter()
    myCopter.initialize()
    myCopter.pos_script(subscriber)

    time.sleep(LAND_TIME)
    print("__exiting script__")
    subscriber.destroy_node

if __name__ == '__main__':
    main()
    