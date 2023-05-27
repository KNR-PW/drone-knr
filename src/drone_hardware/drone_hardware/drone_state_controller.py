import argparse
import time
import math
import rclpy
from rclpy.node import Node

from dronekit import connect, VehicleMode, LocationLocal, LocationGlobalRelative
from pymavlink import mavutil

from drone_interfaces.srv import GetAttitude, GetLocationRelative, SetServo, SetYaw, SetMode


class DroneHandler(Node):
    def __init__(self):
        super().__init__('drone_handler')

        ## DECLARE SERVICES
        self.attitude = self.create_service(GetAttitude, 'get_attitude', self.get_attitude_callback)
        self.gps = self.create_service(GetLocationRelative, 'get_location_relative', self.get_location_relative_callback)
        self.yaw = self.create_service(SetYaw, 'set_yaw', self.set_yaw_callback)
        self.mode = self.create_service(SetMode, 'set_mode',self.set_mode_callback)
       
        ## DRONE MEMBER VARIABLES
        self.state = "BUSY"

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
        
    ## HELPER METHODS
    def set_yaw(self, yaw):
        if yaw<0:
            yaw+=6.283185
        yaw = yaw / 3.141592 * 180
        if abs(self.vehicle.attitude.yaw - yaw) > 3.141592:
            dir = 1 if self.vehicle.attitude.yaw < yaw else -1
        else:
            dir = 1 if self.vehicle.attitude.yaw > yaw else -1
        # create the CONDITION_YAW command using command_long_encode()
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,        # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
            0,           #confirmation
            yaw,         # param 1, yaw in degrees
            0,           # param 2, yaw speed deg/s
            dir,           # param 3, direction -1 ccw, 1 cw
            0, # param 4, relative offset 1, absolute angle 0
            0, 0, 0)     # param 5 ~ 7 not used
        # send command to vehicle
        self.vehicle.send_mavlink(msg)

    ## SERVICE CALLBACKS
    def get_attitude_callback(self, request, response):
        temp = self.vehicle.attitude
        response.roll=temp.roll
        response.pitch=temp.pitch
        response.yaw=temp.yaw
        self.get_logger().info(f"-- Get attitude service called --")
        self.get_logger().info(f"Roll: {response.roll}")
        self.get_logger().info(f"Pitch: {response.pitch}")
        self.get_logger().info(f"Yaw: {response.yaw}")
        return response
    
    def get_location_relative_callback(self, request, response):
        temp = self.vehicle.location.local_frame
        response.north = temp.north
        response.east = temp.east
        response.down = temp.down
        self.get_logger().info(f"-- Get location relative service called --")
        self.get_logger().info(f"North: {response.north}")
        self.get_logger().info(f"East: {response.east}")
        self.get_logger().info(f"Down: {response.down}")
        return response
    
    def set_yaw_callback(self, request, response):
        self.set_yaw(request.yaw)
        response = SetYaw.Response()
        return response
    
    def set_mode_callback(self, request, response):
        self.vehicle.mode = VehicleMode(request.mode)
        response = SetMode.Response()
        return response
    

def main():
    rclpy.init()
    
    drone = DroneHandler()

    rclpy.spin(drone)

    drone.destroy_node()

    rclpy.shutdown()


if __name__ == 'main':
    main()
