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

# transferring files to this branch needed
# from drone_interfaces.srv import GetAttitude, GetLocationRelative, SetServo, SetYaw, SetMode
# from drone_interfaces.action import GotoRelative, GotoGlobal, Arm, Takeoff

class DroneNavigator(Node):
    def __init__(self):
        super().__init__('drone_navigator')

        self.get_logger().info("Hello wrold")


        # ## DECLARE SERVICES
        # self.attitude = self.create_service(GetAttitude, 'get_attitude', self.get_attitude_callback)
        # self.gps = self.create_service(GetLocationRelative, 'get_location_relative', self.get_location_relative_callback)
        # self.servo = self.create_service(SetServo, 'set_servo', self.set_servo_callback)
        # self.yaw = self.create_service(SetYaw, 'set_yaw', self.set_yaw_callback)
        # self.mode = self.create_service(SetMode, 'set_mode',self.set_mode_callback)
        
        # ## DECLARE ACTIONS
        # self.goto_rel = ActionServer(self, GotoRelative, 'goto_relative', self.goto_relative_action)
        # self.goto_global = ActionServer(self, GotoGlobal, 'goto_global', self.goto_global_action)
        # self.arm = ActionServer(self,Arm, 'Arm',self.arm_callback)
        # self.takeoff = ActionServer(self, Takeoff, 'takeoff',self.takeoff_callback)

        # ## DRONE MEMBER VARIABLES
        # self.state = "BUSY"

        # ##CONNECT TO COPTER
        # parser = argparse.ArgumentParser(description='commands')
        # parser.add_argument('--connect', default='127.0.0.1:14550')
        # args = parser.parse_args()

        # connection_string = args.connect

        # sitl = None

        # if not connection_string:
        #     self.get_logger().info("Start simulator (SITL)")
        #     import dronekit_sitl
        #     sitl = dronekit_sitl.start_default()
        #     connection_string = sitl.connection_string()
        # baud_rate = 57600

        # self.vehicle = connect(connection_string, baud=baud_rate, wait_ready=False) #doesnt work with wait_ready=True
        # self.state = "OK"
        # self.get_logger().info("Copter connected, ready to arm")


    # def __del__(self):
    #      self.vehicle.mode = "RTL"

    
    def det2pos(self):
        img_res=np.array((640,480))

        detection=np.array((245,23))
        detection=detection-img_res/2
        detection[1]=-detection[1]

        HFOV=math.radians(62.2)
        VFOV=math.radians(48.8)

        drone_pos = np.array((self.vehicle.location.local_frame.north,self.vehicle.location.local_frame.east))
        drone_yaw = self.vehicle.attitude.yaw
        drone_amplitude = -self.vehicle.location.local_frame.down

        self.cam_range=(math.tan(HFOV)*drone_amplitude,math.tan(VFOV)*drone_amplitude)

        target_pos_rel=np.multiply(np.divide(detection, img_res), self.cam_range)

        self.detections.append(drone_pos+np.matmul(Rot(drone_yaw), target_pos_rel))

        # cam_range1=(math.tan(HFOV)*15,math.tan(VFOV)*15)

        print(self.cam_range)
    

def rotate_vector(north, east, yaw):
    yaw = -yaw # because yaw is clockwise in dronekit
    r = np.matrix([[east],
                    [north]])
    
    rotated = np.matmul(Rot(yaw),r)
    return rotated


def Rot(yaw):
        yaw = math.radians(yaw)
        res = np.matrix([[math.cos(yaw), -math.sin(yaw)], 
                         [math.sin(yaw), math.cos(yaw)]])
        return res

def main():
     rclpy.init()

     drone = DroneNavigator()

     rclpy.spin(drone)

     drone.destroy_node()

     rclpy.shutdown()


if __name__ == "__main__":
     main()