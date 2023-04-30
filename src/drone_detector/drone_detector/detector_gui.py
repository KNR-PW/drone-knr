
import rclpy  # Python library for ROS 2
from PyQt5.QtGui import QPixmap
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import pyqtSignal, pyqtSlot, Qt, QThread
import time
from PyQt5.QtWidgets import QMainWindow, QApplication, QMenu, QAction, QStyle, qApp
from PyQt5.QtCore import Qt, QTimer

import sys

# class DetMainWindow(QtWidgets.QMainWindow):
#     def __init__(self):
#         super().__init__()
#         self.float_topic_name = "/video_frames"
#         self.timer = QtWidgets.QTimer(self)
#         self.timer.timeout.connect(self.timer_image_update)
#         self.br = CvBridge()
#
#     def ros_init(self):
#         rclpy.init(args=None)
#         self.node = Node('detector_gui')
#         self.sub = self.node.create_subscription(
#             Image,
#             self.float_topic_name,
#             self.image_callback,
#             10,
#         )
#         # spin once, timeout_sec 5[s]
#         timeout_sec_rclpy = 5
#         timeout_init = time.time()
#         rclpy.spin_once(self.node, timeout_sec=timeout_sec_rclpy)
#         timeout_end = time.time()
#         ros_connect_time = timeout_end - timeout_init
#         if ros_connect_time >= timeout_sec_rclpy:
#             print("ros connection successful")
#         else:
#             print("ros connection failed")
#     #     Start timer
#         self.timer.start(0.5)
#
#     def timer_image_update(self):
#         rclpy.spin_once(self.node)
#         self.label.setPixmap(self.convert_cv_qt(self.frame))
#         self.timer.start(0.5)
#
#     def image_callback(self, img):
#         frame = self.br.imgmsg_to_cv2(img)
#         self.frame = frame
#
#     def convert_cv_qt(self, cv_img):
#         """Convert from an opencv image to QPixmap"""
#         rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
#         h, w, ch = rgb_image.shape
#         bytes_per_line = ch * w
#         convert_to_Qt_format = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
#         p = convert_to_Qt_format.scaled(self.disply_width, self.display_height, Qt.KeepAspectRatio)
#         return QPixmap.fromImage(p)
class Ui_MainWindow(object):

    def __init__(self):
        self.float_topic_name = "video_frames"
        self.timer = QTimer()
        self.timer.timeout.connect(self.timer_image_update)
        self.disply_width = 640
        self.display_height = 480
        self.br = CvBridge()
        self.frame = 0
        self.got_frame = False

    def ros_shutdown(self):
        self.node.destroy_node()
        rclpy.shutdown()
    def ros_init(self):
        print("ros_init")
        rclpy.init(args=None)
        self.node = Node('detector_gui')
        self.sub = self.node.create_subscription(
            Image,
            self.float_topic_name,
            self.image_callback,
            10,
        )
        # spin once, timeout_sec 5[s]
        timeout_sec_rclpy = 5
        timeout_init = time.time()
        rclpy.spin_once(self.node, timeout_sec=timeout_sec_rclpy)
        timeout_end = time.time()
        ros_connect_time = timeout_end - timeout_init
        if ros_connect_time >= timeout_sec_rclpy:
            print("ros connection successful")
        else:
            print("ros connection failed")
    #     Start timer
        self.timer.start(100)

    def timer_image_update(self):
        print("timer update")
        rclpy.spin_once(self.node, timeout_sec=2)
        if self.got_frame:
            self.label.setPixmap(self.convert_cv_qt(self.frame))
        print("timer update end")

        # self.timer.start(1000)

    def image_callback(self, img):
        print("image callback")
        frame = self.br.imgmsg_to_cv2(img)
        self.got_frame = True
        self.frame = frame

    def convert_cv_qt(self, cv_img):
        """Convert from an opencv image to QPixmap"""
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
        p = convert_to_Qt_format.scaled(self.disply_width, self.display_height, Qt.KeepAspectRatio)
        return QPixmap.fromImage(p)
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1454, 894)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.widget = QtWidgets.QWidget(self.centralwidget)
        self.widget.setGeometry(QtCore.QRect(10, 10, 1431, 831))
        self.widget.setObjectName("widget")
        self.verticalLayout_4 = QtWidgets.QVBoxLayout(self.widget)
        self.verticalLayout_4.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setSizeConstraint(QtWidgets.QLayout.SetNoConstraint)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.label_2 = QtWidgets.QLabel(self.widget)
        self.label_2.setMinimumSize(QtCore.QSize(640, 480))
        self.label_2.setMaximumSize(QtCore.QSize(640, 480))
        self.label_2.setLineWidth(1)
        self.label_2.setText("")
        self.label_2.setPixmap(QtGui.QPixmap("shades.png")) #inserting the photo
        self.label_2.setScaledContents(False)
        self.label_2.setObjectName("label_2")
        self.horizontalLayout.addWidget(self.label_2)
        self.label = QtWidgets.QLabel(self.widget)
        self.label.setMinimumSize(QtCore.QSize(640, 480))
        self.label.setMaximumSize(QtCore.QSize(640, 480))
        self.label.setText("")
        self.label.setPixmap(QtGui.QPixmap("shades 2.png")) #inserting the photo
        self.label.setScaledContents(False)
        self.label.setObjectName("label")
        self.horizontalLayout.addWidget(self.label)
        self.verticalLayout_4.addLayout(self.horizontalLayout)
        self.horizontalLayout_9 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_9.setObjectName("horizontalLayout_9")
        self.horizontalLayout_8 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_8.setObjectName("horizontalLayout_8")
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.horizontalSlider = QtWidgets.QSlider(self.widget)
        self.horizontalSlider.setMaximum(255)
        self.horizontalSlider.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider.setObjectName("horizontalSlider")
        self.horizontalLayout_2.addWidget(self.horizontalSlider)
        self.spinBox = QtWidgets.QSpinBox(self.widget)
        self.spinBox.setMaximum(255)
        self.spinBox.setObjectName("spinBox")
        self.horizontalLayout_2.addWidget(self.spinBox)
        self.verticalLayout.addLayout(self.horizontalLayout_2)
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.horizontalSlider_2 = QtWidgets.QSlider(self.widget)
        self.horizontalSlider_2.setMaximum(255)
        self.horizontalSlider_2.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_2.setObjectName("horizontalSlider_2")
        self.horizontalLayout_3.addWidget(self.horizontalSlider_2)
        self.spinBox_2 = QtWidgets.QSpinBox(self.widget)
        self.spinBox_2.setMaximum(255)
        self.spinBox_2.setObjectName("spinBox_2")
        self.horizontalLayout_3.addWidget(self.spinBox_2)
        self.verticalLayout.addLayout(self.horizontalLayout_3)
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.horizontalSlider_3 = QtWidgets.QSlider(self.widget)
        self.horizontalSlider_3.setMaximum(255)
        self.horizontalSlider_3.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_3.setObjectName("horizontalSlider_3")
        self.horizontalLayout_4.addWidget(self.horizontalSlider_3)
        self.spinBox_3 = QtWidgets.QSpinBox(self.widget)
        self.spinBox_3.setMaximum(255)
        self.spinBox_3.setObjectName("spinBox_3")
        self.horizontalLayout_4.addWidget(self.spinBox_3)
        self.verticalLayout.addLayout(self.horizontalLayout_4)
        self.horizontalLayout_8.addLayout(self.verticalLayout)
        self.verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.horizontalSlider_4 = QtWidgets.QSlider(self.widget)
        self.horizontalSlider_4.setMaximum(255)
        self.horizontalSlider_4.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_4.setObjectName("horizontalSlider_4")
        self.horizontalLayout_5.addWidget(self.horizontalSlider_4)
        self.spinBox_4 = QtWidgets.QSpinBox(self.widget)
        self.spinBox_4.setMaximum(255)
        self.spinBox_4.setObjectName("spinBox_4")
        self.horizontalLayout_5.addWidget(self.spinBox_4)
        self.verticalLayout_2.addLayout(self.horizontalLayout_5)
        self.horizontalLayout_6 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_6.setObjectName("horizontalLayout_6")
        self.horizontalSlider_5 = QtWidgets.QSlider(self.widget)
        self.horizontalSlider_5.setMaximum(255)
        self.horizontalSlider_5.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_5.setObjectName("horizontalSlider_5")
        self.horizontalLayout_6.addWidget(self.horizontalSlider_5)
        self.spinBox_5 = QtWidgets.QSpinBox(self.widget)
        self.spinBox_5.setMaximum(255)
        self.spinBox_5.setObjectName("spinBox_5")
        self.horizontalLayout_6.addWidget(self.spinBox_5)
        self.verticalLayout_2.addLayout(self.horizontalLayout_6)
        self.horizontalLayout_7 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_7.setObjectName("horizontalLayout_7")
        self.horizontalSlider_6 = QtWidgets.QSlider(self.widget)
        self.horizontalSlider_6.setMaximum(255)
        self.horizontalSlider_6.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_6.setObjectName("horizontalSlider_6")
        self.horizontalLayout_7.addWidget(self.horizontalSlider_6)
        self.spinBox_6 = QtWidgets.QSpinBox(self.widget)
        self.spinBox_6.setMaximum(255)
        self.spinBox_6.setObjectName("spinBox_6")
        self.horizontalLayout_7.addWidget(self.spinBox_6)
        self.verticalLayout_2.addLayout(self.horizontalLayout_7)
        self.horizontalLayout_8.addLayout(self.verticalLayout_2)
        self.horizontalLayout_9.addLayout(self.horizontalLayout_8)
        self.verticalLayout_3 = QtWidgets.QVBoxLayout()
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.pushButton = QtWidgets.QPushButton(self.widget)
        self.pushButton.setObjectName("pushButton")
        self.verticalLayout_3.addWidget(self.pushButton)
        self.pushButton_2 = QtWidgets.QPushButton(self.widget)
        self.pushButton_2.setObjectName("pushButton_2")
        self.verticalLayout_3.addWidget(self.pushButton_2)
        self.horizontalLayout_9.addLayout(self.verticalLayout_3)
        self.verticalLayout_4.addLayout(self.horizontalLayout_9)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1454, 21))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)
        self.ros_init()

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.pushButton.setText(_translate("MainWindow", "NEXT"))
        self.pushButton_2.setText(_translate("MainWindow", "OK"))






def main(args=None):
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
    rclpy.shutdown()


if __name__ == '__main__':
    main()