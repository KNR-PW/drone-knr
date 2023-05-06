# latanie irl

from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
import socket
# import exceptions
import math
import argparse

## ==== connection ==== ##

def connectMyCopter():
    parser = argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect', default='127.0.0.1:14550')
    args = parser.parse_args()

    connection_string = args.connect
    baud_rate = 57600

    vehicle = connect(connection_string, baud=baud_rate, wait_ready=False) #doesnt work with wait_ready=True
    return vehicle

def arm():
    while vehicle.is_armable==False:
        print("Waiting for behicle to become armable...")
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



def main():
    global vehicle
    vehicle = connectMyCopter()
    arm()

    vehicle.mode = "GUIDED"

    vehicle.simple_takeoff(10)

    coord1 = LocationGlobalRelative(-35.3615, 149.1636, 10.0)

    vehicle.simple_goto(coord1, 100)


    while vehicle.mode.name == "GUIDED":
        remaining_distance = get_distance_metres(vehicle.location.global_frame, coord1)
        if remaining_distance <= 1:
            print("Reached target waypoint")
            break
        time.sleep(1)
        

    coord2 = LocationGlobalRelative(vehicle.location.global_relative_frame.lat+0.001, vehicle.location.global_relative_frame.lon+0.001, vehicle.location.global_relative_frame.alt)

    vehicle.simple_goto(coord2, 100)

    print("End of script.")

if __name__ == "_main_" :
    main()
