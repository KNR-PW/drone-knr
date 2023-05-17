import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
import numpy as np
# from detection import Detection
from drone_interfaces.msg import DetectionMsg, DetectionsList
from drone_interfaces.srv import DetectTrees
from std_msgs.msg import Int32MultiArray
import time


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


class DetectionClient(Node):

    def __init__(self):
        super().__init__('detection_client')
        self.det_cli = self.create_client(DetectTrees, 'detect_trees')
        while not self.det_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('detect_trees service not available, waiting again...')
        self.req = DetectTrees.Request()

    def send_request(self, info=0):
        self.req.request_info = info
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        print("client recieved response:")
        print(self.future.result().detections_list)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    detection_client = DetectionClient()

    while True:
        detection_client.send_request()
        time.sleep(2)

    detection_client.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
