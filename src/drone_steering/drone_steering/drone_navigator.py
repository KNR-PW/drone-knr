import rclpy
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobal, LocationLocal, LocationGlobalRelative, APIException
import time
import socket
# import exceptions
import math
import argparse
from pymavlink import mavutil # Needed for command message definitions
import haversine as hv # because the dronekit one doesn't work near Poles hehe xD


def get_distance_global(aLocation1, aLocation2):
    coord1 = (aLocation1.lat, aLocation1.lon)
    coord2 = (aLocation2.lat, aLocation2.lon)

    return hv.haversine(coord1, coord2)*1000 # because we want it in metres


def get_distance_metres_ned(aLocation1, aLocation2):
    dnorth = aLocation2.north - aLocation1.north
    deast = aLocation2.east - aLocation1.east
    return math.sqrt((dnorth*dnorth) + (deast*deast))





from drone_interfaces.srv import GetAttitude, GetLocationRelative, SetServo, SetYaw, SetMode
from drone_interfaces.action import GotoRelative, GotoGlobal, GotoLocal, Arm, Takeoff, PhotosTour
from rclpy.node import Node
from rclpy.action import ActionClient
from example_interfaces.srv import AddTwoInts

# TODO DET2POS, PHOTOSTOUR ACTION SERVER

class DroneNavigator(Node):

    def __init__(self):
        super().__init__('drone_navigator')
        # self.srv = self.create_service(
        #     AddTwoInts, 
        #     'add_two_ints', 
        #     self.add_two_ints_callback)

        # self.counter_ = 0
        # self.create_timer(1.0, self.timer_callback)

        # SERVICE CLIENTS
        self.attitude = self.create_client(GetAttitude, 'get_attitude')
        self.gps = self.create_client(GetLocationRelative, 'get_location_relative')
        self.servo = self.create_client(SetServo, 'set_servo')
        self.yaw = self.create_client(SetYaw, 'set_yaw')
        self.mode = self.create_client(SetMode, 'set_mode')

        # ACTION CLIENTS
        self.goto_rel = ActionClient(self, GotoRelative, 'goto_relative')
        self.goto_global = ActionClient(self, GotoGlobal, 'goto_global')
        self.goto_local = ActionClient(self, GotoLocal, 'goto_local')
        self.arm = ActionClient(self, Arm, 'Arm')
        self.takeoff = ActionClient(self, Takeoff, 'takeoff')

        self.photos_tour = ActionClient(self, PhotosTour, 'photos_tour')

        ##CONNECT TO COPTER
        parser = argparse.ArgumentParser(description='commands')
        parser.add_argument('--connect', default='127.0.0.1:14550')
        args = parser.parse_args()

        connection_string = args.connect

        sitl = None

        if not connection_string:
            self.get_logger().info("Start simulator (SITL)")
            import dronekit_sitl
            sitl = dronekit_sitl.start_default()
            connection_string = sitl.connection_string()
        baud_rate = 57600

        self.vehicle = connect(connection_string, baud=baud_rate, wait_ready=False) #doesnt work with wait_ready=True
        self.state = "OK"
        self.get_logger().info("Copter connected, ready to arm")


        # def timer_callback(self):
        #     self.get_logger().info("goodbye " + str(self.counter_))
        #     self.counter_ += 1

        # def add_two_ints_callback(self, request, response):
        #     response.sum = request.a + request.b
        #     self.get_logger().info(
        #         'incoming request\na: %d b: %d' % (request.a, request.b))

        #     return response
        

    def arm_goal(self):
        goal_msg = Arm.Goal()
        
        return self.arm.send_goal(goal_msg)


    # they probably need some update, i don't think they'll work first try
    def goto_global_send_goal(self, aLocation):
        goal_msg = GotoGlobal.Goal()
        goal_msg.lat = aLocation.lat
        goal_msg.lon = aLocation.lon
        goal_msg.alt = aLocation.alt

        self.goto_global.wait_for_server()

        return self.goto_global.send_goal(goal_msg)
    

    def goto_local_send_goal(self, coord):
        goal_msg = GotoLocal.Goal()
        goal_msg.north = coord.north
        goal_msg.east = coord.east
        goal_msg.down = coord.down

        self.goto_local.wait_for_server()

        return self.goto_local.send_goal(goal_msg)


    def goto_rel_send_goal(self, coord):
        goal_msg = GotoRelative.Goal()
        goal_msg.north = coord.north
        goal_msg.east = coord.east
        goal_msg.down = coord.down

        self.goto_rel.wait_for_server()

        return self.goto_rel.send_goal(goal_msg)
    

    def photos_tour_goal(self, length, width):
        goal_msg = PhotosTour.Goal()
        goal_msg.length = length
        goal_msg.width = width

        self.photos_tour.wait_for_server()

        return self.photos_tour.send_goal(goal_msg)



    def __del__(self):
        self.vehicle.mode=VehicleMode("RTL")


def main():
    rclpy.init()

    drone = DroneNavigator()

    rclpy.spin(drone)

    drone.destroy_node()

    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
