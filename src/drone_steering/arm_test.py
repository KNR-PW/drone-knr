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
 

def goto_position_target_global_int(aLocation):
    """
    Send SET_POSITION_TARGET_GLOBAL_INT command to request the vehicle fly to a specified LocationGlobal.

    For more information see: https://pixhawk.ethz.ch/mavlink/#SET_POSITION_TARGET_GLOBAL_INT

    See the above link for information on the type_mask (0=enable, 1=ignore).
    At time of writing, acceleration and yaw bits are ignored.
    """
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111111000, # type_mask (only speeds enabled)
        aLocation.lat*1e7, # lat_int - X Position in WGS84 frame in 1e7 * meters
        aLocation.lon*1e7, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        aLocation.alt, # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
        0, # X velocity in NED frame in m/s
        0, # Y velocity in NED frame in m/s
        0, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to vehicle
    vehicle.send_mavlink(msg)


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


# def goto(dNorth, dEast, gotoFunction =vehicle.simple_goto):
#     """
#     Moves the vehicle to a position dNorth metres North and dEast metres East of the current position.

#     The method takes a function pointer argument with a single `dronekit.lib.LocationGlobal` parameter for 
#     the target position. This allows it to be called with different position-setting commands. 
#     By default it uses the standard method: dronekit.lib.Vehicle.simple_goto().

#     The method reports the distance to target every two seconds.
#     """
    
#     currentLocation = vehicle.location.global_relative_frame
#     targetLocation = get_location_metres(currentLocation, dNorth, dEast)
#     targetDistance = get_distance_metres(currentLocation, targetLocation)
#     gotoFunction(targetLocation)
    
#     #print "DEBUG: targetLocation: %s" % targetLocation
#     #print "DEBUG: targetLocation: %s" % targetDistance

#     while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
#         #print "DEBUG: mode: %s" % vehicle.mode.name
#         remainingDistance=get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
#         print("Distance to target: ", remainingDistance)
#         if remainingDistance<=targetDistance*0.01: #Just below target, in case of undershoot.
#             print("Reached target")
#             break
#         time.sleep(2)


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
