import rclpy
from dronekit import connect, VehicleMode, LocationLocal, LocationGlobalRelative
from pymavlink import mavutil

import argparse
import time
import math

from rclpy.node import Node
from rclpy.action import ActionServer

from drone_interfaces.srv import GetAttitude, GetLocationRelative
from drone_interfaces.action import GotoRelative, GotoGlobal

class DroneHandler(Node):
    def __init__(self):
        super().__init__('drone_handler')

        ## DECLARE SERVICES
        self.attitude = self.create_service(GetAttitude, 'get_attitude', self.get_attitude_callback)
        self.gps = self.create_service(GetLocationRelative, 'get_location_relative', self.get_location_relative_callback)
        
        ## DECLARE ACTIONS
        self.goto_rel = ActionServer(self, GotoRelative, 'goto_relative', self.goto_relative_action)
        self.goto_global = ActionServer(self, GotoGlobal, 'goto_global', self.goto_global_action)

        ## DRONE MEMBER VARIABLES
        self.state = "BUSY"
        self.current_destination_rel = LocationLocal()
        self.current_destination_global = LocationGlobalRelative()

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

    def __del__(self):
        self.vehicle.mode=VehicleMode("RTL")


    ## INTERNAL HELPER METHODS
    def arm(self):
        self.state = "BUSY"
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
        self.state = "OK"
    
    def takeoff(self, altitude):
        self.state = "OK"
        self.vehicle.simple_takeoff(altitude)

        # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
        #  after Vehicle.simple_takeoff will execute immediately).
        while True:
            self.get_logger().info(f"Altitude: {self.vehicle.location.global_relative_frame.alt}")
            if self.vehicle.location.global_relative_frame.alt >= altitude * 0.97:  # Trigger just below target alt.
                self.get_logger().info("Reached target altitude")
                break
            time.sleep(1)
        self.state = "OK"

    def goto_position_target_local_ned(self, north, east, down):
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

    
    def calculate_remaining_distance_rel(self, location):
        dnorth = location.north - self.current_destination_rel.north
        deast = location.east - self.current_destination_rel.east
        ddown = location.down - self.current_destination_rel.down
        return math.sqrt(dnorth*dnorth+deast*deast+ddown*ddown)
    
    def calculate_remaining_distance_global(self, location):
        dlat = (location.lat - self.current_destination_global.lat) * 1.113195e5 ## lat/lon to meters convert magic number
        dlon = (location.lon - self.current_destination_global.lon) * 1.113195e5 ## lat/lon to meters convert magic number
        ddown = location.down - self.current_destination_global.down
        return math.sqrt(dlat * dlat + dlon * dlon + ddown * ddown)

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

    
    ## ACTION CALLBACKS
    def goto_relative_action(self, goal_handle):
        self.get_logger().info(f'Flying to: lat={goal_handle.request}')
        self.vehicle.simple_goto()



        goal_handle.succeed()
        result=GotoRelative.Result()
        # result.distance = distance
        return result.roll, result.pitch, result.yaw
    
    def goto_global_action(self, goal_handle):
        return 



def main():
    rclpy.init()
    
    drone = DroneHandler()

    rclpy.spin(drone)

    drone.destroy_node()

    rclpy.shutdown()


if __name__ == 'main':
    main()