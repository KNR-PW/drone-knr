import rclpy
from dronekit import connect, VehicleMode, LocationLocal, LocationGlobalRelative
from pymavlink import mavutil

import argparse
import time
import math

from rclpy.node import Node
from rclpy.action import ActionServer

from drone_interfaces.srv import GetAttitude, GetLocationRelative, SetServo, SetYaw, SetMode
from drone_interfaces.action import GotoRelative, GotoGlobal, Arm, Takeoff, Shoot

class DroneHandler(Node):
    def __init__(self):
        super().__init__('drone_handler')

        ## DECLARE SERVICES
        self.attitude = self.create_service(GetAttitude, 'get_attitude', self.get_attitude_callback)
        self.gps = self.create_service(GetLocationRelative, 'get_location_relative', self.get_location_relative_callback)
        self.servo = self.create_service(SetServo, 'set_servo', self.set_servo_callback)
        self.yaw = self.create_service(SetYaw, 'set_yaw', self.set_yaw_callback)
        self.mode = self.create_service(SetMode, 'set_mode',self.set_mode_callback)
        self.shoot = ActionServer(self, Shoot, 'shoot', self.shoot_callback)
        ## DECLARE ACTIONS
        self.goto_rel = ActionServer(self, GotoRelative, 'goto_relative', self.goto_relative_action)
        self.goto_global = ActionServer(self, GotoGlobal, 'goto_global', self.goto_global_action)
        self.arm = ActionServer(self,Arm, 'Arm',self.arm_callback)
        self.takeoff = ActionServer(self, Takeoff, 'takeoff',self.takeoff_callback)

        ## DRONE MEMBER VARIABLES
        self.state = "BUSY"

        ##CONNECT TO COPTER


        connection_string = "/dev/serial0"
        baud_rate = 921600
        self.get_logger().info("Connecting with copter...")
        self.vehicle = connect(connection_string, baud=baud_rate, wait_ready=False) #doesnt work with wait_ready=True
        self.state = "OK"
        self.get_logger().info("Copter connected, ready to arm")

    def __del__(self):
        self.vehicle.mode=VehicleMode("RTL")

    def shoot_callback(self, goal_handle):
        stop = 1000
        shoot = 1200
        load =1500

        left = 2100
        mid = 1400
        right = 800

        self.set_servo(10,stop)
        self.set_servo(11,stop)
        time.sleep(1)
        self.set_servo(9,left if goal_handle.request.color == 'yellow' else right)
        self.set_servo(10,load)
        self.set_servo(11,load)
        time.sleep(2)
        self.set_servo(10,shoot)
        self.set_servo(11,shoot)
        self.set_servo(9,mid - 300 if goal_handle.request.color == 'yellow' else mid+300)
        time.sleep(1)
        self.set_servo(10,stop)
        self.set_servo(11,stop)
        self.set_servo(9,mid)

        self.get_logger().info("Shoot action completed:" + goal_handle.request.color)
        goal_handle.succeed()
        result = Shoot.Result()
        return result
        
    ## INTERNAL HELPER METHODS
    def goto_position_target_local_ned(self, north, east, down=-1):
        if down == -1:
            down = self.vehicle.location.local_frame.down
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
            0b0000111111111000, # type_mask (only positions enabled)
            north, east, down, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
            0, 0, 0, # x, y, z velocity in m/s  (not used)
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        # send command to vehicle
        self.vehicle.send_mavlink(msg)

    def goto_position_target_global_int(self, location):
        msg = self.vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111111000, # type_mask (only speeds enabled)
        location.lat*1e7, # lat_int - X Position in WGS84 frame in 1e7 * meters
        location.lon*1e7, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        location.alt, # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
        0, # X velocity in NED frame in m/s
        0, # Y velocity in NED frame in m/s
        0, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        # send command to vehicle
        self.vehicle.send_mavlink(msg)

    def set_yaw(self, yaw, relative=False):
        if yaw<0:
            yaw+=6.283185
        yaw = yaw / 3.141592 * 180
        if abs(self.vehicle.attitude.yaw - yaw) > 3.141592:
            dir = 1 if self.vehicle.attitude.yaw < yaw else -1
        else:
            dir = 1 if self.vehicle.attitude.yaw > yaw else -1
        if relative:
            is_relative=1 #yaw relative to direction of travel
        else:
            is_relative=0 #yaw is an absolute angle
        # create the CONDITION_YAW command using command_long_encode()
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,        # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
            0,           #confirmation
            yaw,         # param 1, yaw in degrees
            0,           # param 2, yaw speed deg/s
            1,           # param 3, direction -1 ccw, 1 cw
            is_relative, # param 4, relative offset 1, absolute angle 0
            0, 0, 0)     # param 5 ~ 7 not used
        # send command to vehicle
        self.vehicle.send_mavlink(msg)

    def set_servo(self, servo_id, pwm):
        msg = self.vehicle.message_factory.command_long_encode(
            0,          # time_boot_ms (not used)
            0,   # target system, target component
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO, #command
            0,          #not used
            servo_id,   #number of servo instance
            pwm,        #pwm value for servo control
            0,0,0,0,0) #not used
        # send command to vehicle
        self.vehicle.send_mavlink(msg)

    def calculate_remaining_distance_rel(self, location):
        dnorth = location.north - self.vehicle.location.local_frame.north
        deast = location.east - self.vehicle.location.local_frame.east
        ddown = location.down - self.vehicle.location.local_frame.down
        return math.sqrt(dnorth*dnorth + deast*deast + ddown*ddown)
    
    def calculate_remaining_distance_global(self, location):
        dlat = (location.lat - self.vehicle.location.global_relative_frame.lat) * 1.113195e5 ## lat/lon to meters convert magic number
        dlon = (location.lon - self.vehicle.location.global_relative_frame.lon) * 1.113195e5 ## lat/lon to meters convert magic number
        ddown = location.down - self.vehicle.location.global_relative_frame.down
        return math.sqrt(dlat*dlat + dlon*dlon + ddown*ddown)

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
        response.north = temp.north or 0.0
        response.east = temp.east or 0.0
        response.down = temp.down or 0.0
        # self.get_logger().info(f"-- Get location relative service called --")
        # self.get_logger().info(f"North: {response.north}")
        # self.get_logger().info(f"East: {response.east}")
        # self.get_logger().info(f"Down: {response.down}")
        return response
    
    def set_yaw_callback(self, request, response):
        self.set_yaw(request.yaw)
        response = SetYaw.Response()
        return response

    def set_servo_callback(self, request, response):
        self.set_servo(request.servo_id, request.pwm)
        response = SetServo.Response()
        return response
    
    def set_mode_callback(self, request, response):
        self.vehicle.mode = VehicleMode(request.mode)
        response = SetMode.Response()
        return response

    ## ACTION CALLBACKS
    def goto_relative_action(self, goal_handle):
        self.get_logger().info(f'-- Goto relative action registered. Destination in local frame: --')

        north = self.vehicle.location.local_frame.north + goal_handle.request.north
        east = self.vehicle.location.local_frame.east + goal_handle.request.east
        down = self.vehicle.location.local_frame.down + goal_handle.request.down
        destination = LocationLocal(north, east, down)

        self.get_logger().info(f'North: {destination.north}')
        self.get_logger().info(f'East: {destination.east}')
        self.get_logger().info(f'Down: {destination.down}')

        self.state = "BUSY"

        self.goto_position_target_local_ned(destination.north, destination.east, destination.down)

        feedback_msg = GotoRelative.Feedback()
        feedback_msg.distance = self.calculate_remaining_distance_rel(destination)
        self.get_logger().info(f"Distance remaining: {feedback_msg.distance} m")

        while feedback_msg.distance>0.5:
            feedback_msg.distance = self.calculate_remaining_distance_rel(destination)
            self.get_logger().info(f"Distance remaining: {feedback_msg.distance} m")
            time.sleep(1)

        goal_handle.succeed()
        self.state = "OK"
        result = GotoRelative.Result()
        result.result=1

        return result
    
    def goto_global_action(self, goal_handle):
        self.get_logger().info(f'-- Goto global action registered. Destination in global frame: --')

        lat = self.vehicle.location.global_relative_frame.lat + goal_handle.request.lat
        lon = self.vehicle.location.global_relative_frame.lon + goal_handle.request.lon
        alt = self.vehicle.location.global_relative_frame.alt + goal_handle.request.alt
        destination=LocationGlobalRelative(lat,lon,alt)

        self.get_logger().info(f'Latitude: {destination.lat}')
        self.get_logger().info(f'Longitude: {destination.lon}')
        self.get_logger().info(f'Altitude: {destination.alt}')

        self.state = "BUSY"

        self.goto_position_target_local_ned(destination.north, destination.east, destination.alt)

        feedback_msg = GotoGlobal.Feedback()
        feedback_msg.distance = self.calculate_remaining_distance_rel(destination)
        self.get_logger().info(f"Distance remaining: {feedback_msg.distance} m")

        while feedback_msg.distance>0.5:
            feedback_msg.distance = self.calculate_remaining_distance_global(destination)
            self.get_logger().info(f"Distance remaining: {feedback_msg.distance} m")
            time.sleep(1)

        goal_handle.succeed()
        self.state = "OK"
        result = GotoGlobal.Result()
        result.result = 1

        return result
    
    def arm_callback(self, goal_handle):
        self.get_logger().info(f'-- Arm action registered --')
        self.state = "BUSY"
        feedback_msg = Arm.Feedback()
        
        while self.vehicle.is_armable==False:
            feedback_msg.feedback = "Waiting for vehicle to become armable..."
            self.get_logger().info(feedback_msg.feedback)
            time.sleep(1)

        self.vehicle.armed=True
        while self.vehicle.armed==False:
            feedback_msg.feedback = "Waiting for drone to become armed..."
            self.get_logger().info(feedback_msg.feedback)
            time.sleep(1)

        feedback_msg.feedback = "Vehicle is now armed."
        self.get_logger().info(feedback_msg.feedback)

        self.state = "OK"
        
        goal_handle.succeed()
        self.state = "OK"
        result = Arm.Result()
        result.result = 1

        return result
    
    def takeoff_callback(self, goal_handle):
        feedback_msg = Takeoff.Feedback()

        self.state = "OK"
        self.vehicle.simple_takeoff(goal_handle.request.altitude)

        # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
        #  after Vehicle.simple_takeoff will execute immediately).
        while self.vehicle.location.global_relative_frame.alt >= goal_handle.request.altitude * 0.97:
            feedback_msg.altitude = self.vehicle.location.global_relative_frame.alt
            self.get_logger().info(f"Altitude: {feedback_msg.altitude}")
            time.sleep(1)

        self.get_logger().info("Reached target altitude")
        
        goal_handle.succeed()
        self.state = "OK"
        result = Takeoff.Result()
        result.result = 1

        return result

    def publish_gps_pos(self):
        gps_msg = GPSPos()
        GPSPos.gps_position = vehicle.location.local_frame
        self.gps_publisher.piblish(GPSPos)

def main():
    rclpy.init()
    
    drone = DroneHandler()

    rclpy.spin(drone)

    drone.destroy_node()

    rclpy.shutdown()


if __name__ == 'main':
    main()