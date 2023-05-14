import rclpy
from dronekit import connect

import argparse
import time
import math

from rclpy.node import Node
from rclpy.action import ActionServer

from drone_interfaces.srv import GetYaw
from drone_interfaces.action import GotoPos

class DroneHandler(Node):
    def __init__(self):
        super().__init__('drone_handler')

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
        while self.vehicle.is_armable==False:
            self.get_logger().info("Waiting for vehicle to become armable...")
            time.sleep(1)
        self.get_logger().info("Vehicle is now armable")

        #self.vehicle.armed=True # nie wywala bledow dzieki temu potem
        while self.vehicle.armed==False:
            self.get_logger().info("Waiting for drone to become armed...")
            time.sleep(1)

        self.get_logger().info("Vehicle is now armed.")
        self.get_logger().info("props are spinning!")

        ## DECLARE ACTIONS AND SERVICES
        self.yaw = self.create_service(
            GetYaw,
            'get_yaw_orientation',
            self.get_yaw_callback)
        
        self.goto = ActionServer(
            self,
            GotoPos,
            'goto_pos',
            self.goto_action
        )

    
    def get_yaw_callback():
        pass
    
    def goto_action():
        pass



def main():
    rclpy.init()
    
    drone = DroneHandler()

    rclpy.spin(drone)

    drone.destroy_node()

    rclpy.shutdown()


if __name__ == 'main':
    main()