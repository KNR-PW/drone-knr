import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
import numpy as np
# from detection import Detection
from drone_interfaces.msg import DetectionMsg, DetectionsList

class DetectionSubscriber(Node):
    """
    Create an ImagePublisher class, which is a subclass of the Node class.
    """


    def __init__(self):
        super().__init__('detection_subscriber')
        self.detection_subscription = self.create_subscription(
            DetectionsList,
            'detections',
            self.detection_callback,
            10)
        self.image_subscription = self.create_subscription(
            Image,
            'camera',
            self.image_callback,
            10)
        self.br = CvBridge()
        self.detections = []
        self.frame = 0
        # self.detection_msg = Detection()
        self.detections_list_msg = DetectionsList()
        self.get_logger().info('DetectionPublisher node created')

    def image_callback(self, frame):
        # self.get_logger().info('Received frame detections list')
        # Convert ROS Image message to OpenCV image

        frame = self.br.imgmsg_to_cv2(frame)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        self.frame = frame



    def detection_callback(self, detections):
        # self.get_logger().info('Received detections list')
        self.get_logger().info(detections.detections_list[0].color_name)
        for det in detections.detections_list:
            x, y, w, h = det.bounding_box
            cv2.rectangle(self.frame, (x, y),
                        (x + h, y + w),
                        (0, 255, 0), 5)
        cv2.imshow("detector", self.frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    detector_subscriber = DetectionSubscriber()

    rclpy.spin(detector_subscriber)

    detector_subscriber.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
