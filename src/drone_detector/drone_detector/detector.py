import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
import numpy as np
# from detection import Detection
from drone_interfaces.msg import Detection, DetectionsList


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
class Detector(Node):
    """
    Create an ImagePublisher class, which is a subclass of the Node class.
    """


    def __init__(self):
        super().__init__('detector')
        self.subscription = self.create_subscription(
            Image,
            'camera',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(DetectionsList, 'detections', 10)
        timer_period = 2
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.br = CvBridge()
        self.thresholds = {"brown": (np.array([50,80,100 ]), np.array([80, 110,140 ])),
                           "beige": (np.array([0,0,140]), np.array([100, 100, 255])),
                           "golden": (np.array([0,0,140]), np.array([100, 100, 255]))}
        self.detections = []
        # self.detection_msg = Detection()
        self.detections_list_msg = DetectionsList()
        self.get_logger().info('Detector node created')

    def listener_callback(self, frame):
        self.get_logger().info('Receiving video frame and detecting')
        self.detections.clear()
        # Display the message on the console

        # Convert ROS Image message to OpenCV image
        frame = self.br.imgmsg_to_cv2(frame)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        for col in self.thresholds:
            thres = self.thresholds[col]
            mask = cv2.inRange(frame, thres[0], thres[1])
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                # Calculate area and remove small elements
                area = cv2.contourArea(cnt)
                if area > 100:
                    x, y, w, h = cv2.boundingRect(cnt)
                    self.detections.append(Detection(bounding_box=(x, y, w, h), color=col))
                    cv2.rectangle(frame, (x, y),
                                  (x + h, y + w),
                                  (0, 255, 0), 5)
    #     Convert detections to ros msg
    #     self.detections_to_msg()
        cv2.imshow("detector", frame)
        cv2.waitKey(1)

    def timer_callback(self):
        self.get_logger().info('Publishing detections list')
        self.publisher.publish(self.detections_list_msg)

    def detections_to_msg(self):
        detection_msg = Detection()
        temp_detection_list_msg = DetectionsList()
        for detection in self.detections:
            detection_msg.bounding_box = [detection.get_bounding_box()[0], detection.get_bounding_box()[1]]
            detection_msg.color = detection.get_color()
            detection_msg.gps_pos = detection.get_gps_pos()
            temp_detection_list_msg.append(detection_msg)


def main(args=None):
    rclpy.init(args=args)

    detector = Detector()

    rclpy.spin(detector)

    detector.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
