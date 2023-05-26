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

from rclpy.node import Node
from rclpy.action import ActionClient

from example_interfaces.srv import AddTwoInts


class DroneNavigator(Node):

    def __init__(self):
        super().__init__('drone_navigator')
        # self.srv = self.create_service(
        #     AddTwoInts, 
        #     'add_two_ints', 
        #     self.add_two_ints_callback)

        # self.counter_ = 0
        # self.create_timer(1.0, self.timer_callback)

        # ACTION CLIENTS
        self.goto_rel = ActionClient(self, GotoRelative, 'goto_relative')
        self.goto_global = ActionClient(self, GotoGlobal, 'goto_global', self.goto_global_action)
        self.arm = ActionClient(self, Arm, 'Arm', self.arm_callback)
        self.takeoff = ActionClient(self, Takeoff, 'takeoff', self.takeoff_callback)


    # def timer_callback(self):
    #     self.get_logger().info("goodbye " + str(self.counter_))
    #     self.counter_ += 1

    # def add_two_ints_callback(self, request, response):
    #     response.sum = request.a + request.b
    #     self.get_logger().info(
    #         'incoming request\na: %d b: %d' % (request.a, request.b))

    #     return response


def main():
    rclpy.init()

    drone = DroneNavigator()

    rclpy.spin(drone)

    # drone.destroy_node()

    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
