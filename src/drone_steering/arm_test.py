from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobal, LocationLocal, LocationGlobalRelative, APIException
import time
import socket
# import exceptions
import math
import argparse
from pymavlink import mavutil # Needed for command message definitions


# TODO 
# do ogarniecia loty po kolejnych kolkach, funkcje np kolko lewo prawo, oblot po wszystkich kolkach
# przerzucenie wszystkiego do obiektu misja

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


# alternatively


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


# get gps:
# vehicle.location.global_relative_frame

# coordinates for this function need to be passed like this:
# a_location = LocationGlobalRelative(-5, 5, -5)



# spits out the distance between two given points
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
 


def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned LocationGlobal has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.

    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.

    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius = 6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    if type(original_location) is LocationGlobal:
        targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
    else:
        raise Exception("Invalid Location object passed")
        
    return targetlocation

# in global coordinate system, like you'd typically give someone coordinates
# could be useful if our camera gave us coordinates in 'gps' style
def goto_position_target_local_ned(north, east, down):
    """
    Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified
    location in the North, East, Down frame.

    It is important to remember that in this frame, positive altitudes are entered as negative
    "Down" values. So if down is "10", this will be 10 metres below the home altitude.

    Starting from AC3.3 the method respects the frame setting. Prior to that the frame was
    ignored. For more information see:
    http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned

    See the above link for information on the type_mask (0=enable, 1=ignore).
    At time of writing, acceleration and yaw bits are ignored.

    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to vehicle
    vehicle.send_mavlink(msg)



def pos_change(north, east, down):

    while not vehicle.gps_0.fix_type:
        print("waiting for positional info")
        time.sleep(1)

    # Get the vehicle's current position in the local frame of reference
    current_location = vehicle.location.local_frame

    # next_location = LocationGlobal

    # next_location.lat = current_location.lat + north
    # next_location.lon = current_location.lon + east
    # next_location.alt = current_location.alt + down

    # goto_position_target_local_ned(next_location.lat, next_location.lon, next_location.alt)
    goto_position_target_local_ned(
        vehicle.location.local_frame.north+north,
         vehicle.location.local_frame.east+east, 
         vehicle.location.local_frame.down+down)
    time.sleep(10)


    # Get the vehicle's current position in the local frame of reference
    current_location = vehicle.location.local_frame

    # Print the position
    print('Current position (local frame): {}'.format(current_location))


def main():
    global vehicle #not cool
    vehicle = connectMyCopter()

    arm_and_takeoff(10)

    # # steering in a global Frame of Reference
    # coord1 = LocationGlobalRelative(-35.3630, 149.1630, 5)
    
    # vehicle.simple_goto(coord1)

    # while vehicle.mode == "GUIDED":
    #     remaining_distance = get_distance_metres(vehicle.location.global_frame, coord1)
    #     if remaining_distance <= 1:
    #         print("Reached target waypoint")
    #         break
    #     time.sleep(1)


    # NED coordinates - north, east, down
    pos_change(10,0,0)
    pos_change(-5,-5,-1)


    print("End of script.")


if __name__ == "__main__":
    main()
