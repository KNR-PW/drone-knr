import rclpy
from dronekit import connect, VehicleMode

import argparse
import time
import math

from rclpy.node import Node
from rclpy.action import ActionServer

from drone_interfaces.srv import GetAttitude
from drone_interfaces.action import GotoPos

class DroneHandler(Node):
    def __init__(self):
        super().__init__('drone_handler')

        ## DECLARE ACTIONS AND SERVICES
        self.attitude = self.create_service(GetAttitude, 'get_attitude', self.get_attitude_callback)
        
        self.goto = ActionServer(
            self,
            GotoPos,
            'goto_pos',
            self.goto_action
        )

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
        self.get_logger().info("Copter connected, ready to take instructions")

        ## ARM COPTER
        self.vehicle.mode=VehicleMode("GUIDED")
        while self.vehicle.is_armable==False:
            self.get_logger().info("Waiting for vehicle to become armable...")
            time.sleep(5)
        self.get_logger().info("Vehicle is now armable")

        self.vehicle.armed=True
        while self.vehicle.armed==False:
            self.get_logger().info("Waiting for drone to become armed...")
            time.sleep(1)

        self.get_logger().info("Vehicle is now armed.")
        self.get_logger().info("props are spinning!")

    def __del__(self):
        self.vehicle.mode=VehicleMode("RTL")


    
    def get_attitude_callback(self, request, response):
        response.roll=self.vehicle.attitude.roll
        response.pitch=self.vehicle.attitude.pitch
        response.yaw=self.vehicle.attitude.yaw
        self.get_logger().info(f"Get Attitude service called")
        self.get_logger().info(f"Roll: {response.roll}")
        self.get_logger().info(f"Pitch: {response.pitch}")
        self.get_logger().info(f"Yaw: {response.yaw}")
        return response

    
    def goto_action(self, goal_handle):
        self.get_logger().info(f'Flying to: lat={goal_handle.request}')
        self.vehicle.simple_goto()



        goal_handle.succeed()
        result=GotoPos.Result()
        # result.distance = distance
        return result.roll, result.pitch, result.yaw



def main():
    rclpy.init()
    
    drone = DroneHandler()

    rclpy.spin(drone)

    drone.destroy_node()

    rclpy.shutdown()


if __name__ == 'main':
    main()