from __future__ import print_function
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobal, LocationLocal, LocationGlobalRelative, APIException
import time
import socket
# import exceptions
import math
import argparse
from pymavlink import mavutil # Needed for command message definitions


global vehicle # CANNOT BE THAT WAY


class DroneMission:
    def __init__(self):
        # connecting

        parser = argparse.ArgumentParser(description='commands')
        parser.add_argument('--connect', default='127.0.0.1:14550')
        args = parser.parse_args()

        connection_string = args.connect

        # if we don't pass any connection string, it runs sitl
        sitl = None

        if not connection_string:
            print("Start simulator (SITL)")
            import dronekit_sitl
            sitl = dronekit_sitl.start_default()
            connection_string = sitl.connection_string()
        
        baud_rate = 57600
        self.vehicle = connect(connection_string, baud=baud_rate, wait_ready=False) #doesnt work with wait_ready=True

        return None


    def __str__(self):
        return None
    
    
    def arm_and_takeoff(self, aTargetAltitude):
        """
        Arms vehicle and flies to aTargetAltitude.
        """

        print("Basic pre-arm checks")
        # Don't let the user try to arm until autopilot is ready
        while not self.vehicle.is_armable:
            print(" Waiting for vehicle to become armable...")
            time.sleep(1)
        print("Vehicle is now armable.")
        print("")

        print("Arming motors")
        # Copter should arm in GUIDED mode
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True # wtedy sie nie wywala

        while not self.vehicle.armed:
            print(" Waiting for arming...")
            time.sleep(1)

        print("Taking off!")
        self.vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

        # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
        #  after Vehicle.simple_takeoff will execute immediately).
        while True:
            print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)
            if self.vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:  # Trigger just below target alt.
                print("Reached target altitude")
                break
            time.sleep(1)

        return None
    

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


    def pos_change(self, north, east, down):

        while not self.vehicle.gps_0.fix_type:
            print("waiting for positional info")
            time.sleep(1)

        # Get the vehicle's current position in the local frame of reference
        current_location = self.vehicle.location.local_frame

        next_location = LocationLocal

        next_location.north = current_location.north + north
        next_location.east = current_location.east + east
        next_location.down = current_location.down + down

        self.goto_position_target_local_ned(next_location.north, next_location.east, next_location.down)
        
        while self.vehicle.mode == "GUIDED":
            remaining_distance = get_distance_metres_ned(self.vehicle.location.local_frame, next_location)
            if remaining_distance <= 1:
                print("Reached target waypoint")
                break
            time.sleep(1)

        # Get the vehicle's current position in the local frame of reference
        current_location = self.vehicle.location.local_frame

        # Print the position
        print('Current position (local frame): {}'.format(current_location))


    def goto_position_ned(self, coord = LocationLocal):
        self.goto_position_target_local_ned(coord.north, coord.east, coord.down)


    def next_circle(self, circle_pos = LocationLocal):
        pass
    
    
# spits out the distance between two given points in global frame
def get_distance_metres(aLocation1, aLocation2):
    dlat = aLocation2.lat - aLocation1.lat 
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


# spits out the distance between two given points in local frame
def get_distance_metres_ned(aLocation1, aLocation2):
    dnorth = aLocation2.north - aLocation1.north
    deast = aLocation2.east - aLocation1.east
    return math.sqrt((dnorth*dnorth) + (deast*deast))


def main():
    drone = DroneMission()

    drone.arm_and_takeoff(10)


    # drone.pos_change(5,-22,0)
    # drone.pos_change(-5,-5,-1)

    coord1 = LocationLocal
    coord1.north = 9
    coord1.east = -14
    coord1.down = -15

    drone.goto_position_ned(coord1)

    print("End of script.")
    
    # TODO na ten moment by się przydało napisać funkcje takie które mogą się przydać do oblatywania drona
    #   chyba spoko byłoby zmontować wstępny sposób nadlatywania nad grupy drzew


if __name__ == "__main__":
    main()