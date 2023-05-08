# latanie irl
from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, APIException
import time
import socket
# import exceptions
import math
import argparse
from pymavlink import mavutil # Needed for command message definitions



## ==== connection ==== ##

def connectMyCopter():
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


def arm():
    while vehicle.is_armable==False:
        print("Waiting for vehicle to become armable...")
        time.sleep(1)
    print("Vehicle is now armable")
    print("")

    vehicle.armed=True # nie wywala bledow dzieki temu potem
    while vehicle.armed==False:
        print("Waiting for drone to become armed...")
        time.sleep(1)

    print("Vehicle is now armed.")
    print("props are spinning!")

    return None


# get gps:
# vehicle.location.global_relative_frame

# coordinates for this function need to be passed like this:
# a_location = LocationGlobalRelative(-21, 37, 69)


def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """

    dlat = aLocation2.lat - aLocation1.lat 
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


def main():
    global vehicle
    vehicle = connectMyCopter()
    arm()

    vehicle.mode = "GUIDED"

    vehicle.simple_takeoff(5)

    time.sleep(10)

    coord1 = LocationGlobalRelative(-35.3636, 149.1630, 5)
    
    vehicle.simple_goto(coord1)


    while vehicle.mode == "GUIDED":
        remaining_distance = get_distance_metres(vehicle.location.global_frame, coord1)
        if remaining_distance <= 1:
            print("Reached target waypoint")
            break
        time.sleep(1)
        

    coord2 = LocationGlobalRelative(vehicle.location.global_relative_frame.lat+0.001, vehicle.location.global_relative_frame.lon+0.001, vehicle.location.global_relative_frame.alt)

    vehicle.simple_goto(coord2, 10)

    while vehicle.mode == "GUIDED":
        remaining_distance = get_distance_metres(vehicle.location.global_frame, coord2)
        if remaining_distance <= 1:
            print("Reached target waypoint")
            break
        time.sleep(1)

    coord3 = LocationGlobalRelative(vehicle.location.global_relative_frame.lat-0.0001, vehicle.location.global_relative_frame.lon+0.0001, vehicle.location.global_relative_frame.alt)

    vehicle.simple_goto(coord3, 10)

    print("End of script.")

if __name__ == "__main__":
    main()
