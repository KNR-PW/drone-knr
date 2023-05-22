from __future__ import print_function
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobal, LocationLocal, LocationGlobalRelative, APIException
import time
import socket
# import exceptions
import math
import argparse
from pymavlink import mavutil # Needed for command message definitions
import haversine as hv # because the dronekit one doesn't work near Poles hehe xD
import cv2

class DroneMission:
    def __init__(self):
        # connecting

        self.detections=list()

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
    

    def condition_yaw(self, heading, relative=False): # set relative to true if you want to set it relatively to current yaw
        if relative:
            is_relative = 1 #yaw relative to direction of travel
        else:
            is_relative = 0 #yaw is an absolute angle
        # create the CONDITION_YAW command using command_long_encode()
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
            0, #confirmation
            heading,    # param 1, yaw in degrees
            0,          # param 2, yaw speed deg/s
            1,          # param 3, direction -1 ccw, 1 cw
            is_relative, # param 4, relative offset 1, absolute angle 0
            0, 0, 0)    # param 5 ~ 7 not used
        # send command to vehicle
        self.vehicle.send_mavlink(msg)


    def goto_position_target_global_int(self, aLocation):
        msg = self.vehicle.message_factory.set_position_target_global_int_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
            0b0000111111111000, # type_mask (only speeds enabled)
            int(aLocation.lat*1e7), # lat_int - X Position in WGS84 frame in 1e7 * meters
            int(aLocation.lon*1e7), # lon_int - Y Position in WGS84 frame in 1e7 * meters
            aLocation.alt, # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
            0, # X velocity in NED frame in m/s
            0, # Y velocity in NED frame in m/s
            0, # Z velocity in NED frame in m/s
            0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
        # send command to vehicle
        self.vehicle.send_mavlink(msg)


    # above, but improved with ability to finish before next function starts
    def goto_global(self, aLocation):
        self.goto_position_target_global_int(aLocation)

        while self.vehicle.mode == "GUIDED":
            remaining_distance = get_dist(self.vehicle.location.global_frame, aLocation)
            if remaining_distance <= 0.25:
                print("Reached target waypoint")
                break
            time.sleep(1)


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
            if remaining_distance <= 0.25:
                print("Reached target waypoint")
                break
            time.sleep(1)

        # Get the vehicle's current position in the local frame of reference
        current_location = self.vehicle.location.local_frame

        # Print the position
        print('Current position (local frame): {}'.format(current_location))


    def goto_position_ned(self, coord = LocationLocal):
        self.goto_position_target_local_ned(coord.north, coord.east, coord.down)

        while self.vehicle.mode == "GUIDED":
            remaining_distance = get_distance_metres_ned(self.vehicle.location.local_frame, coord)
            if remaining_distance <= 0.25:
                print("Reached target waypoint")
                break
            time.sleep(1)


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

        cam_n_l = (int(self.cam_range[0]/4))+1 # camera number of circles, length
        print("longer side camera circles: ", cam_n_l)

        cam_n_w = (int(self.cam_range[1]/4))+1 # camera number of circles, width
        print("shorter side camera circles: ", cam_n_w)

        cam_circles = cam_n_l*cam_n_w
        print("camera circles for attitude of ", round(-self.vehicle.location.local_frame.down), " metres is: ", cam_circles)

        # ekf origin czy home

        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        

    # vectors for one side, in local system - let's say we fly through the points (given in global) and pass them into here:
    def local_circles(self, l_coordrd, l_coordld, l_coordlu):
        length = get_distance_metres_ned(l_coordrd, l_coordld)
        width = get_distance_metres_ned(l_coordld, l_coordlu)

        print("length local: ", length)
        print("width local: ", width)

        # if length > width:
        #     self.condition_yaw(current_yaw, False)
        # else:
        #     self.condition_yaw(current_yaw+90, False)

        n_length = (round(length/4))+1
        print("number of circles from right to left: ", n_length)

        n_width = (round(width/4))+1
        print("number of circles from down to up: ", n_width)

        n_circles = n_width*n_length

        if n_circles != 100:
            print("wrong number of circles, something's wrong")


        cam_range_l = self.cam_range[0]-1 # one meter cut for better accuracy
        cam_range_w = self.cam_range[1]-1

        l_tours = round(length/cam_range_l)
        w_tours = round(width/cam_range_w)

        current_yaw = self.vehicle.attitude.yaw

        delta = rotate_vector(cam_range_l, cam_range_w, current_yaw)

        self.pos_change(-delta[1,0]/2, -delta[0,0]/2, 0) # move from the edge of map, delta(1) is y, so north
        
        k = 1

        for i in range(w_tours):
            for j in range(l_tours):
                self.pos_change(k*-delta[1,0], 0, 0)

                # taking a photo, detection and flying to the circles here

            self.pos_change(0, -delta[0,0], 0)
            k = -k
        


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


# spits out the distance between two given points in global frame
def get_distance_metres(aLocation1, aLocation2):
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


# using haversine formula, above doesn't work in australia
def get_dist(aLocation1, aLocation2):
    coord1 = (aLocation1.lat, aLocation1.lon)
    coord2 = (aLocation2.lat, aLocation2.lon)

    return hv.haversine(coord1, coord2)*1000 # because we want it in metres


# spits out the distance between two given points in local frame
def get_distance_metres_ned(aLocation1, aLocation2):
    dnorth = aLocation2.north - aLocation1.north
    deast = aLocation2.east - aLocation1.east
    return math.sqrt((dnorth*dnorth) + (deast*deast))


def next_circle(self, circle_pos = LocationLocal):
    pass


def main():
    drone = DroneMission()

    altit = 10
    drone.arm_and_takeoff(altit)

    time.sleep(2)

    coordrd = LocationGlobal(lat=-35.3632186,lon=149.1650381,alt=altit)
    coordld = LocationGlobal(lat=-35.3628949,lon=149.165038,alt=altit)
    coordlu = LocationGlobal(lat=-35.3628948,lon=149.165435,alt=altit)

    drone.goto_global(coordrd)
    time.sleep(1)
    coord1 = drone.vehicle.location.local_frame # rd

    # # drone.pos_change(0,-16,0)
    # # home = LocationLocal(north=5, east = -18, down = -alt)
    # # drone.goto_position_ned(home)

    drone.goto_global(coordld)
    time.sleep(1)
    coord2 = drone.vehicle.location.local_frame #ld

    # # time.sleep(3)
    # # print("right down pos: ", drone.vehicle.location.global_frame)

    drone.goto_global(coordlu)
    time.sleep(1)
    coord3 = drone.vehicle.location.local_frame # lu

    drone.det2pos()

    drone.local_circles(coord1, coord2, coord3)

    drone.vehicle.mode = "RTL"

    print("End of script.")

    # 1 wszy algorytm to lista punktów
    # przeliczenie tmmem, przeliczenie tak na local frame


    # 2gi algorytm to środki zdjęć jakoś

    # 1wszy algorytm to tworzenie listy zdjec
    # i to obliczamy przy obrocie o dany kąt tak żeby minimalizować pole tego szukanego tego
    # z wektora

    # 3 algorytm to chore punkty i zestrzelenie

    # wypierdalac te powielone tym dystansem, albo średnią z dwóch punktów

    # po zrobieniu zdjec puszczmay serwis do detektora on robi zdjecie i zwraca kolejną kolejkę z tymi do zestrzelenia



if __name__ == "__main__":
    main()