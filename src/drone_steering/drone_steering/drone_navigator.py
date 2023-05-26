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
from rclpy.action import ActionServer


class DroneNavigator(Node):
    def __init__(self):
        super().__init__('drone_navigator')

        self.get_logger().info("hello z dupy")


def main():
    rclpy.init()

    drone = DroneNavigator()

    rclpy.spin(drone)

    drone.destroy_node()

    rclpy.shutdown()
    


if __name__ == '__main__':
    main()
