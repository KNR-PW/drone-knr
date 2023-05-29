import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobal, LocationLocal, LocationGlobalRelative, APIException
# from detection import Detection
from drone_interfaces.msg import DetectionMsg, DetectionsList
from drone_interfaces.srv import DetectTrees, GetLocationRelative, GetAttitude, SetYaw
from drone_interfaces.action import GotoRelative, GotoGlobal
from std_msgs.msg import Int32MultiArray
import time
from rclpy.action import ActionClient
import haversine as hv
import math

class Detection:
    def __init__(self, bounding_box=(0, 0, 0, 0), color="", gps_pos=(0, 0)):
        # Format x, y, w, h
        self.bounding_box = bounding_box
        self.color = color
        self.gps_pos = gps_pos

    def set_bounding_box(self, bb):
        self.bounding_box = bb

    def set_gps_pos(self, pos):
        self.gps_pos = pos

    def get_bounding_box(self):
        return self.bounding_box

    def get_gps_pos(self):
        return self.gps_pos

    def get_color(self):
        return self.color

    def add_image(self, image):
        pass


class Mission(Node):
    def __init__(self):
        super().__init__("mission")
        self.det_cli = self.create_client(DetectTrees, "detect_trees")
        while not self.det_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "detect_trees service not available, waiting again..."
            )
        self.req = DetectTrees.Request()
        self.gps_cli = self.create_client(GetLocationRelative, "get_location_relative")
        while not self.gps_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("GPS service not available, waiting again...")
        self.atti_cli = self.create_client(GetAttitude, "get_attitude")
        while not self.atti_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("attitude service not available, waiting again...")
        self.goto_rel_action_client = ActionClient(self, GotoRelative, "goto_relative")
        self.goto_glob_action_client = ActionClient(self, GotoGlobal, "goto_global")
        self.yaw_cli = self.create_client(SetYaw, "set_yaw")
        while not self.yaw_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("set yaw service not available, waiting again...")
        self.get_logger().info("GotoDetectionGroup node created")
        self.state = "OK"
        altit = 10
        self.coordrd = LocationGlobal(lat=-35.3632183,lon=149.1654352,alt=altit)
        self.coordld = LocationGlobal(lat=-35.3632186,lon=149.1650381,alt=altit)
        self.coordlu = LocationGlobal(lat=-35.3628949,lon=149.165038,alt=altit)
        self.coordru = LocationGlobal(lat=-35.3628948,lon=149.165435,alt=altit)
        self.local_coordru = 0
        self.local_coordrd = 0
        self.local_coordld = 0
        self.local_coordlu = 0
        self.current_yaw = 0
        self.length = 0 
        self.width = 0
        self.scan_altitude = 10
        self.shoot_altitude = 5

    def set_area_coords(self, coordru, coordrd, coordld, coordlu):
        self.coordlu = coordlu
        self.coordrd = coordrd
        self.coordld = coordld
        self.coordru = coordru
        self.get_logger().info("Area coordinates set")

    def send_set_yaw(self, yaw):
        request = SetYaw.Request()
        request.yaw = float(yaw)
        self.yaw_cli.call_async(request)
        self.get_logger().info("Yaw request sent")

    def send_detection_request(self, info=0, gps=[0.0, 0.0, 0.0], yaw=0.0):
        gps = [float(x) for x in gps]
        self.get_logger().info("Sending detection request")
        self.req.request_info = info
        self.req.gps = gps
        self.req.yaw = float(yaw)
        future = self.det_cli.call_async(self.req)
        print("called")
        rclpy.spin_until_future_complete(self, future, timeout_sec=10)
        self.get_logger().info("Client recieved detection response")
        # print(future.result().detections_list)
        return future.result().detections_list.detections_list

    def goto_det_group(self, det_list):
        relative_move = [0, 0]
        for det in det_list:
            self.get_logger().info("Going to next det")
            # self.send_set_yaw(self.current_yaw)
            # time.sleep(3)
            gps_position = det.gps_position
            self.send_goto_relative(
                gps_position[0] - relative_move[0],
                gps_position[1] - relative_move[1],
                0.0,
            )
            relative_move[0] += gps_position[0]
            relative_move[1] += gps_position[1]
            while self.state == "BUSY":
                # self.get_logger().info("Waiting for goto det")
                # time.sleep(1)
                rclpy.spin_once(self, timeout_sec=0.05)
        return 1

    def get_gps(self):
        self.get_logger().info("Sending GPS request")
        request_gps = GetLocationRelative.Request()
        gps_future = self.gps_cli.call_async(request_gps)
        # gps_future.add_done_callback(self.gps_get_result)
        rclpy.spin_until_future_complete(self, gps_future, timeout_sec=5)
        if gps_future.result() is not None:
            self.north = gps_future.result().north
            self.east = gps_future.result().east
            self.down = gps_future.result().down
            self.drone_amplitude = -self.down
            self.get_logger().info("GPS Recieved")
        else:
            self.get_logger().info("GPS request failed")
            se
            self.drone_amplitude = 0
        return [self.north, self.east, self.down]

    def get_yaw(self):
        self.get_logger().info("Sending yaw request")
        request_attitude = GetAttitude.Request()
        atti_future = self.atti_cli.call_async(request_attitude)
        rclpy.spin_until_future_complete(self, atti_future, timeout_sec=5)
        yaw = atti_future.result().yaw
        return yaw

    def send_goto_relative(self, north, east, down):
        self.state = "BUSY"
        self.get_logger().info("Sending goto relative action goal")
        goal_msg = GotoRelative.Goal()
        goal_msg.north = float(north)
        goal_msg.east = float(east)
        goal_msg.down = float(down)
        while not self.goto_rel_action_client.wait_for_server():
            self.get_logger().info("waiting for goto server...")

        self.send_goal_future = self.goto_rel_action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goto_rel_response_callback)
        self.get_logger().info("Goto action sent")

    def goto_rel_response_callback(self, future):
        self.get_logger().info("Goto rel response callback")
        goal_handle = future.result()
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.goto_rel_result_callback)

    def goto_rel_result_callback(self, future):
        self.get_logger().info("Goto rel  action finished")
        self.state = "OK"

    def send_goto_global(self, lat, lon, alt):
        self.state = "BUSY"
        self.get_logger().info("Sending goto global action goal")
        goal_msg = GotoGlobal.Goal()
        goal_msg.lat = float(lat)
        goal_msg.lon = float(lon)
        goal_msg.alt = float(alt)
        while not self.goto_rel_action_client.wait_for_server():
            self.get_logger().info("waiting for goto server...")

        self.send_goal_future = self.goto_glob_action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goto_glob_response_callback)
        self.get_logger().info("Goto action sent")

    def goto_glob_response_callback(self, future):
        self.get_logger().info("Goto rel response callback")
        goal_handle = future.result()
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.goto_glob_result_callback)

    def goto_glob_result_callback(self, future):
        self.get_logger().info("Goto rel  action finished")
        self.state = "OK"

    def photos_tour(self):
        length = self.length
        width = self.width
        # CHOOSE HOW MUCH TO OVERLAP PHOTOS HERE
        HFOV=math.radians(62.2)
        VFOV=math.radians(48.8)
        cam_range=(math.tan(HFOV/2)*self.scan_altitude*2,math.tan(VFOV/2)*self.scan_altitude*2)
        cam_range_l = cam_range[0]-4 # two meters cut for better accuracy and overlapping
        cam_range_w = cam_range[1]-4

        # add 0.5 to always round up
        l_tours = round((self.length/cam_range_l)+0.5)
        w_tours = round((self.width/cam_range_w)+0.5)
        self.get_logger().info(f"cam_range_l {cam_range_l}, cam_range_w {cam_range_w}")
        self.get_logger().info(f"l_tours {l_tours}, w_tours {w_tours}")

        delta = self.rotate_vector(cam_range_l, cam_range_w, self.current_yaw) #delta is camera's range's vector, it's then rotated to match the field
        self.get_logger().info(f"delta: {delta}")
        self.get_logger().info(f"delta 0: {delta[0,0]},  delta 1: {delta[1,0]}")
        self.send_goto_relative(-delta[1,0]/2, -delta[0,0]/2, 0) # move from the edge of map, delta(1) is y, so north
        
        k = 1

        # if length > width:
        #     current_yaw = current_yaw + 3.141592/2
        #     print("dupa")

        for i in range(w_tours):
            self.get_logger().info("Next w tour")
            for j in range(l_tours-1):
                self.get_logger().info("Next l tour")
                # self.set_yaw(current_yaw)
                time.sleep(4)
                self.send_goto_relative(k*-delta[1,0], 0, 0)
                while self.state == "BUSY":
                    # self.get_logger().info("Waiting for goto action")
                    rclpy.spin_once(self, timeout_sec=0.05)
            
                
                # taking a photo, detection and flying to the circles here

            if i < w_tours-1:
                # self.set_yaw(current_yaw)
                time.sleep(4)
                self.send_goto_relative(0, -delta[0,0], 0)
                while self.state == "BUSY":
                    # self.get_logger().info("Waiting for goto action")
                    rclpy.spin_once(self, timeout_sec=0.05)
                k = -k

    def scan_area(self):
        self.get_logger().info("Scanning area")
        
        self.send_goto_global(lat=self.coordrd.lat, lon=self.coordrd.lon, alt=self.coordrd.alt)
        while self.state == "BUSY":
            rclpy.spin_once(self, timeout_sec=0.1)
        self.local_coordrd = self.get_gps()
        self.get_logger().info(f"Local coordinate recieved, ru:  {self.local_coordrd}")

        self.send_goto_global(lat=self.coordld.lat, lon=self.coordld.lon, alt=self.coordld.alt)
        while self.state == "BUSY":
            rclpy.spin_once(self, timeout_sec=0.1)

        self.local_coordld = self.get_gps()
        self.get_logger().info(f"Local coordinate recieved, ru:  {self.local_coordld}")


        self.send_goto_global(lat=self.coordlu.lat, lon=self.coordlu.lon, alt=self.coordlu.alt)
        while self.state == "BUSY":
            rclpy.spin_once(self, timeout_sec=0.1)
        self.local_coordlu = self.get_gps()
        self.get_logger().info(f"Local coordinate recieved, ru:  {self.local_coordlu}")
    
        
        self.send_goto_global(lat=self.coordru.lat, lon=self.coordru.lon, alt=self.coordru.alt)
        while self.state == "BUSY":
            rclpy.spin_once(self, timeout_sec=0.1)
        self.local_coordru = self.get_gps()
        self.get_logger().info(f"Local coordinate recieved, ru:  {self.local_coordru}")
        # SCAN IS ALWAYS FINISHED IN RU
        
        self.circles_calc()
        self.current_yaw = self.get_yaw()


        
    def get_distance_metres_ned(self, aLocation1, aLocation2):
        dnorth = aLocation2[0] - aLocation1[0]
        deast = aLocation2[1] - aLocation1[1]
        return math.sqrt((dnorth*dnorth) + (deast*deast))
        
       
    def circles_calc(self):
        l_coordrd = self.local_coordrd
        l_coordru = self.local_coordru
        l_coordld = self.local_coordld
        l_coordlu = self.local_coordlu

        length_d = self.get_distance_metres_ned(l_coordrd, l_coordld) # down and up
        length_u = self.get_distance_metres_ned(l_coordru, l_coordlu)
        
        width_r = self.get_distance_metres_ned(l_coordrd, l_coordru) #right
        width_l = self.get_distance_metres_ned(l_coordld, l_coordlu) #left

        # Change with and length for nore logical sense
        self.width = (length_d+length_u)/2
        self.length = (width_r+width_l)/2

        print("length local: ", self.length)
        print("width local: ", self.width)

    def rotate_vector(self, north, east, yaw):
            yaw = -yaw # because yaw is clockwise in dronekit
            r = np.matrix([[east],
                        [north]])
            
            rotated = np.matmul(self.Rot(yaw),r)
            return rotated


    def Rot(self, yaw):
            yaw = math.radians(yaw)
            res = np.matrix([[math.cos(yaw), -math.sin(yaw)], 
                            [math.sin(yaw), math.cos(yaw)]])
            return res


    # using haversine formula, dronekit's doesn't work in some cases
    def get_distance_global(self, aLocation1, aLocation2):
        coord1 = (aLocation1.lat, aLocation1.lon)
        coord2 = (aLocation2.lat, aLocation2.lon)

        return hv.haversine(coord1, coord2)*1000 # because we want it in metres


    # spits out the distance between two given points in local frame
    # def get_distance_metres_ned(self, aLocation1, aLocation2):
    #     dnorth = aLocation2.north - aLocation1.north
    #     deast = aLocation2.east - aLocation1.east
    #     return math.sqrt((dnorth*dnorth) + (deast*deast))

def main(args=None):
    rclpy.init(args=args)

    mission = Mission()
    mission.scan_area()
    mission.photos_tour()
    # for i in range(1):
    #     gps = mission.get_gps()
    #     yaw = mission.get_yaw()
    #     det_list = mission.send_detection_request(gps=gps, yaw=yaw)
    #     print(mission.goto_det_group(det_list))
    mission.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
