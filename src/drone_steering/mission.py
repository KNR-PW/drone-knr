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
    def __init__(self, length, width, vehicle):
        self.length = length
        self.width = width


    def __str__(self):
        return f"Mission length: {self.length}, mission width: {self.width}"
    

    def connectMyCopter(self):
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
        vehicle = connect(connection_string, baud=baud_rate, wait_ready=False) #doesnt work with wait_ready=True
        return vehicle
    
    
    def arm_and_takeoff(aTargetAltitude):
        """
        Arms vehicle and flies to aTargetAltitude.
        """

        print("Basic pre-arm checks")
        # Don't let the user try to arm until autopilot is ready
        while not vehicle.is_armable:
            print(" Waiting for vehicle to become armable...")
            time.sleep(1)
        print("Vehicle is now armable.")
        print("")

        print("Arming motors")
        # Copter should arm in GUIDED mode
        vehicle.mode = VehicleMode("GUIDED")
        vehicle.armed = True # wtedy sie nie wywala

        while not vehicle.armed:
            print(" Waiting for arming...")
            time.sleep(1)

        print("Taking off!")
        vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

        # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
        #  after Vehicle.simple_takeoff will execute immediately).
        while True:
            print(" Altitude: ", vehicle.location.global_relative_frame.alt)
            if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:  # Trigger just below target alt.
                print("Reached target altitude")
                break
            time.sleep(1)

        return None


    def create_map(self):
        map = np.empty(self.length, self.width, 2) #2 bo muszą być 2 współrzędne

        for i in range(self.length):
            for j in range(self.width):
                    map[i][j][0] = i*4 # because they're 4m apart
                    map[i][j][1] = j*4

        for i in range(self.length):
            for j in range(self.width):
                    print("x: ", map[i][j][0], ", y: ", map[i][j][1])
        #probably third attribute about health


    def detection(self):
        pass



def main():


    orchard = DroneMission(25, 4)
    orchard.create_map()


    print(orchard)

    # TODO na ten moment by się przydało napisać funkcje takie które mogą się przydać do oblatywania drona
    #   chyba spoko byłoby zmontować wstępny sposób nadlatywania nad grupy drzew


if __name__ == "__main__":
    main()