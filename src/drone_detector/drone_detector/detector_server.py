import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
import numpy as np
# from detection import Detection
from drone_interfaces.msg import DetectionMsg, DetectionsList
from drone_interfaces.srv import DetectTrees, TakePhoto
from std_msgs.msg import Int32MultiArray
import os


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


class DetectorServer(Node):

    def __init__(self):
        super().__init__('detector_server')
        self.thresholds_subscription = self.create_subscription(Int32MultiArray,
                                                                "detector_thresholds",
                                                                self.thresholds_callback,
                                                                10)

        self.detections_srv = self.create_service(DetectTrees, 'detect_trees', self.detect_trees_callback)
        self.photo_svr = self.create_service(TakePhoto, 'take_photo', self.take_photo_callback)
        self.br = CvBridge()
        self.thresholds = {"brown": (np.array([50, 80, 100]), np.array([80, 110, 140])),
                           "beige": (np.array([0, 0, 140]), np.array([100, 100, 255])),
                           "golden": (np.array([0, 0, 140]), np.array([100, 100, 255]))}
        self.detections = []
        self.img_size = (640, 480)
        self.series_counter = 0
        self.photos_path = "/home/stas/Dron/drone_photos/"
        # self.detection_msg = Detection()
        self.detections_list_msg = DetectionsList()

        self.get_logger().info('DetectorServer node created')
        self.video_capture = cv2.VideoCapture(0)
        while (self.video_capture.isOpened() == False):
            self.get_logger().info('Waiting for camera video cpture to open...')
        _, self.frame = self.video_capture.read()

    def take_photo_callback(self, request, response):
        photos_number = request.photos_number
        print(os.path.abspath(__file__))
        for i in range(photos_number):
            ret, frame = self.video_capture.read()
            if ret == 0:
                self.get_logger().info('Taking photo failed')
                break
            else:
                print(self.photos_path + "drone_photo" + str(self.series_counter) + str(i) + '.jpg')
                cv2.imwrite(self.photos_path + "drone_photo" + str(self.series_counter) + str(i) + '.jpg', frame)

        self.series_counter += 1
        self.get_logger().info(f'Taking  {photos_number} photos succeeded')

        return response

    def detect_trees_callback(self, request, response):
        self.get_logger().info('Incoming detection request')
        self.read_frame()  # TODO
        self.detection(self.frame)
        self.detections_to_msg()
        response.detections_list = self.detections_list_msg
        self.get_logger().info('Response generated. Found trees: ' + str(len(self.detections)))

        return response

    def thresholds_callback(self, msg):
        thres_array = msg.data
        col_arr = ["brown", "beige", "golden"]
        col = col_arr[thres_array[0] - 1]
        self.thresholds[col] = (np.array([thres_array[1], thres_array[2], thres_array[3]]),
                                np.array([thres_array[4], thres_array[5], thres_array[6]]))
        print(self.thresholds)

    def detection(self, frame):
        # self.get_logger().info('Receiving video frame and detecting')
        self.detections.clear()

        # Detection
        for col in self.thresholds:
            thres = self.thresholds[col]
            mask = cv2.inRange(frame, thres[0], thres[1])
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                # Calculate area and remove small elements
                area = cv2.contourArea(cnt)
                if area > 200:
                    x, y, w, h = cv2.boundingRect(cnt)
                    self.detections.append(Detection(bounding_box=(x, y, w, h), color=col))

    def read_frame(self):
        ret, frame = self.video_capture.read()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame = cv2.resize(frame, self.img_size, interpolation=cv2.INTER_AREA)

    def detections_to_msg(self):
        temp_detection_list_msg = DetectionsList()
        self.detections_list_msg.detections_list.clear()

        for detection in self.detections:
            detection_msg = DetectionMsg()
            detection_msg.bounding_box = [detection.get_bounding_box()[0],
                                          detection.get_bounding_box()[1],
                                          detection.get_bounding_box()[2],
                                          detection.get_bounding_box()[3]]
            detection_msg.color_name = detection.get_color()
            detection_msg.gps_position = [detection.get_gps_pos()[0], detection.get_gps_pos()[1]]

            temp_detection_list_msg.detections_list.append(detection_msg)

        self.detections_list_msg.detections_list = temp_detection_list_msg.detections_list


def main(args=None):
    rclpy.init(args=args)

    detector_server = DetectorServer()

    rclpy.spin(detector_server)

    detector_server.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
