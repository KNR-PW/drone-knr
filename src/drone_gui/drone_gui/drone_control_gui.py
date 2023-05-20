import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

import sys
import time
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QMessageBox
from rcl_interfaces.msg import Log
from PyQt5.QtCore import Qt, QTimer
from drone_interfaces.action import GotoRelative, Takeoff, Arm
from drone_interfaces.srv import SetYaw, GetAttitude, GetLocationRelative, SetMode, SetServo


class Ui_MainWindow(object):
    def __init__(self):
        self.timer = QTimer()
        self.timer.timeout.connect(self.timer_ros_update)

    def ros_init(self):
        rclpy.init(args=None)
        self.node = Node('drone_control_gui')
        self.ros_init_clients()
        self.log_subscription = self.node.create_subscription(
            Log,
            '/rosout',
            self.log_callback,
            10)

        # spin once, timeout_sec 5[s]
        timeout_sec_rclpy = 5
        timeout_init = time.time()
        rclpy.spin_once(self.node, timeout_sec=timeout_sec_rclpy)
        timeout_end = time.time()
        ros_connect_time = timeout_end - timeout_init
        if ros_connect_time >= timeout_sec_rclpy:
            self.node.get_logger().info("Ros connection succesfull")
        else:
            self.node.get_logger().info("Ros connection failed")
        self.timer.start(100)

    def ros_init_clients(self):
        ## DECLARE SERVICES
        self.attitude_cli = self.node.create_client(GetAttitude, 'get_attitude')
        self.gps_cli = self.node.create_client(GetLocationRelative, 'get_location_relative')
        self.servo_cli = self.node.create_client(SetServo, 'set_servo')
        self.yaw_cli = self.node.create_client(SetYaw, 'set_yaw')
        self.mode_cli = self.node.create_client(SetMode, 'set_mode')

        ## DECLARE ACTIONS
        self.node.goto_rel_action_client = ActionClient(self.node, GotoRelative, 'goto_relative')
        self.node.arm_action_client = ActionClient(self.node, Arm, 'Arm')
        self.node.takeoff_action_client = ActionClient(self.node, Takeoff, 'takeoff')

    def ros_send_goto_relative(self, north, east, down):
        self.node.get_logger().info("Sending goto relative action goal")
        goal_msg = GotoRelative.Goal()
        goal_msg.north = north
        goal_msg.east = east
        goal_msg.down = down
        while not self.node.goto_rel_action_client.wait_for_server():
            self.get_logger().info('waiting for goto server...')

        self.node.goto_rel_action_client.send_goal_async(goal_msg)
        self.node.get_logger().info("Goto action sent")

    def timer_ros_update(self):
        rclpy.spin_once(self.node, timeout_sec=0.05)

    def log_callback(self, log):
        self.textBrowser.append(log.name + ": " + log.msg)

    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1285, 706)
        MainWindow.setStyleSheet("background-color: rgb(45,40,80);\n"
                                 "color: rgb(240,240,240);")
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.label_4 = QtWidgets.QLabel(self.centralwidget)
        self.label_4.setGeometry(QtCore.QRect(30, 10, 731, 71))
        font = QtGui.QFont()
        font.setPointSize(30)
        font.setBold(True)
        font.setWeight(75)
        self.label_4.setFont(font)
        self.label_4.setAlignment(QtCore.Qt.AlignCenter)
        self.label_4.setObjectName("label_4")
        self.label_5 = QtWidgets.QLabel(self.centralwidget)
        self.label_5.setGeometry(QtCore.QRect(80, 130, 301, 61))
        font = QtGui.QFont()
        font.setPointSize(24)
        font.setBold(True)
        font.setWeight(75)
        self.label_5.setFont(font)
        self.label_5.setAlignment(QtCore.Qt.AlignCenter)
        self.label_5.setObjectName("label_5")
        self.gridLayoutWidget_2 = QtWidgets.QWidget(self.centralwidget)
        self.gridLayoutWidget_2.setGeometry(QtCore.QRect(440, 210, 321, 281))
        self.gridLayoutWidget_2.setObjectName("gridLayoutWidget_2")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.gridLayoutWidget_2)
        self.gridLayout_2.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.pushButton_9 = QtWidgets.QPushButton(self.gridLayoutWidget_2)
        self.pushButton_9.setEnabled(True)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pushButton_9.sizePolicy().hasHeightForWidth())
        self.pushButton_9.setSizePolicy(sizePolicy)
        self.pushButton_9.setText("")
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap("../../../../../Downloads/left.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.pushButton_9.setIcon(icon)
        self.pushButton_9.setIconSize(QtCore.QSize(20, 20))
        self.pushButton_9.setObjectName("pushButton_9")
        self.gridLayout_2.addWidget(self.pushButton_9, 1, 0, 1, 1)
        self.pushButton_2 = QtWidgets.QPushButton(self.gridLayoutWidget_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pushButton_2.sizePolicy().hasHeightForWidth())
        self.pushButton_2.setSizePolicy(sizePolicy)
        self.pushButton_2.setText("")
        icon1 = QtGui.QIcon()
        icon1.addPixmap(QtGui.QPixmap("../../../../../Downloads/right-arrow-svgrepo-com.svg"), QtGui.QIcon.Normal,
                        QtGui.QIcon.Off)
        self.pushButton_2.setIcon(icon1)
        self.pushButton_2.setObjectName("pushButton_2")
        self.gridLayout_2.addWidget(self.pushButton_2, 1, 2, 1, 1)
        self.pushButton_5 = QtWidgets.QPushButton(self.gridLayoutWidget_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pushButton_5.sizePolicy().hasHeightForWidth())
        self.pushButton_5.setSizePolicy(sizePolicy)
        self.pushButton_5.setText("")
        icon2 = QtGui.QIcon()
        icon2.addPixmap(QtGui.QPixmap("../../../../../Downloads/down.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.pushButton_5.setIcon(icon2)
        self.pushButton_5.setObjectName("pushButton_5")
        self.gridLayout_2.addWidget(self.pushButton_5, 3, 1, 1, 1)
        self.pushButton_6 = QtWidgets.QPushButton(self.gridLayoutWidget_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pushButton_6.sizePolicy().hasHeightForWidth())
        self.pushButton_6.setSizePolicy(sizePolicy)
        self.pushButton_6.setText("")
        icon3 = QtGui.QIcon()
        icon3.addPixmap(QtGui.QPixmap("../../../../../Downloads/up.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.pushButton_6.setIcon(icon3)
        self.pushButton_6.setObjectName("pushButton_6")
        self.gridLayout_2.addWidget(self.pushButton_6, 0, 1, 1, 1)
        self.textBrowser = QtWidgets.QTextBrowser(self.centralwidget)
        self.textBrowser.setGeometry(QtCore.QRect(780, 20, 471, 551))
        self.textBrowser.setStyleSheet("background-color: rgb(40,30,55);\n"
                                       "")
        self.textBrowser.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.textBrowser.setObjectName("textBrowser")
        self.gridLayoutWidget_3 = QtWidgets.QWidget(self.centralwidget)
        self.gridLayoutWidget_3.setGeometry(QtCore.QRect(450, 510, 301, 81))
        self.gridLayoutWidget_3.setObjectName("gridLayoutWidget_3")
        self.gridLayout_3 = QtWidgets.QGridLayout(self.gridLayoutWidget_3)
        self.gridLayout_3.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_3.setObjectName("gridLayout_3")
        self.pushButton_8 = QtWidgets.QPushButton(self.gridLayoutWidget_3)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pushButton_8.sizePolicy().hasHeightForWidth())
        self.pushButton_8.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setPointSize(13)
        font.setBold(True)
        font.setWeight(75)
        self.pushButton_8.setFont(font)
        self.pushButton_8.setIcon(icon3)
        self.pushButton_8.setObjectName("pushButton_8")
        self.gridLayout_3.addWidget(self.pushButton_8, 0, 0, 1, 1)
        self.pushButton_7 = QtWidgets.QPushButton(self.gridLayoutWidget_3)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pushButton_7.sizePolicy().hasHeightForWidth())
        self.pushButton_7.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setPointSize(13)
        font.setBold(True)
        font.setWeight(75)
        self.pushButton_7.setFont(font)
        self.pushButton_7.setIcon(icon2)
        self.pushButton_7.setObjectName("pushButton_7")
        self.gridLayout_3.addWidget(self.pushButton_7, 0, 1, 1, 1)
        self.gridLayoutWidget_4 = QtWidgets.QWidget(self.centralwidget)
        self.gridLayoutWidget_4.setGeometry(QtCore.QRect(30, 210, 395, 391))
        self.gridLayoutWidget_4.setObjectName("gridLayoutWidget_4")
        self.gridLayout_4 = QtWidgets.QGridLayout(self.gridLayoutWidget_4)
        self.gridLayout_4.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_4.setObjectName("gridLayout_4")
        self.north_lineEdit = QtWidgets.QLineEdit(self.gridLayoutWidget_4)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.north_lineEdit.sizePolicy().hasHeightForWidth())
        self.north_lineEdit.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setPointSize(19)
        self.north_lineEdit.setFont(font)
        self.north_lineEdit.setStyleSheet("background-color: rgb(40,30,55);")
        self.north_lineEdit.setInputMask("")
        self.north_lineEdit.setText("")
        self.north_lineEdit.setObjectName("north_lineEdit")
        self.gridLayout_4.addWidget(self.north_lineEdit, 0, 1, 1, 1)
        self.pushButton = QtWidgets.QPushButton(self.gridLayoutWidget_4)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pushButton.sizePolicy().hasHeightForWidth())
        self.pushButton.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setPointSize(25)
        font.setBold(True)
        font.setWeight(75)
        self.pushButton.setFont(font)
        self.pushButton.setObjectName("pushButton")
        self.gridLayout_4.addWidget(self.pushButton, 3, 0, 1, 3)
        self.label_3 = QtWidgets.QLabel(self.gridLayoutWidget_4)
        font = QtGui.QFont()
        font.setPointSize(20)
        font.setBold(True)
        font.setWeight(75)
        self.label_3.setFont(font)
        self.label_3.setAlignment(QtCore.Qt.AlignCenter)
        self.label_3.setIndent(5)
        self.label_3.setObjectName("label_3")
        self.gridLayout_4.addWidget(self.label_3, 2, 0, 1, 1)
        self.down_lineEdit = QtWidgets.QLineEdit(self.gridLayoutWidget_4)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.down_lineEdit.sizePolicy().hasHeightForWidth())
        self.down_lineEdit.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setPointSize(19)
        self.down_lineEdit.setFont(font)
        self.down_lineEdit.setStyleSheet("background-color: rgb(40,30,55);")
        self.down_lineEdit.setObjectName("down_lineEdit")
        self.gridLayout_4.addWidget(self.down_lineEdit, 2, 1, 1, 1)
        self.label_6 = QtWidgets.QLabel(self.gridLayoutWidget_4)
        font = QtGui.QFont()
        font.setPointSize(20)
        font.setBold(True)
        font.setWeight(75)
        self.label_6.setFont(font)
        self.label_6.setAlignment(QtCore.Qt.AlignCenter)
        self.label_6.setIndent(5)
        self.label_6.setObjectName("label_6")
        self.gridLayout_4.addWidget(self.label_6, 0, 0, 1, 1)
        self.label = QtWidgets.QLabel(self.gridLayoutWidget_4)
        font = QtGui.QFont()
        font.setPointSize(20)
        font.setBold(True)
        font.setWeight(75)
        self.label.setFont(font)
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setIndent(5)
        self.label.setObjectName("label")
        self.gridLayout_4.addWidget(self.label, 1, 0, 1, 1)
        self.east_lineEdit = QtWidgets.QLineEdit(self.gridLayoutWidget_4)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.east_lineEdit.sizePolicy().hasHeightForWidth())
        self.east_lineEdit.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setPointSize(19)
        self.east_lineEdit.setFont(font)
        self.east_lineEdit.setStyleSheet("background-color: rgb(40,30,55);")
        self.east_lineEdit.setText("")
        self.east_lineEdit.setObjectName("lineEdit")
        self.gridLayout_4.addWidget(self.east_lineEdit, 1, 1, 1, 1)
        self.horizontalLayoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.horizontalLayoutWidget.setGeometry(QtCore.QRect(450, 610, 301, 51))
        self.horizontalLayoutWidget.setObjectName("horizontalLayoutWidget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget)
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.label_2 = QtWidgets.QLabel(self.horizontalLayoutWidget)
        font = QtGui.QFont()
        font.setPointSize(15)
        font.setBold(True)
        font.setWeight(75)
        self.label_2.setFont(font)
        self.label_2.setAlignment(QtCore.Qt.AlignCenter)
        self.label_2.setObjectName("label_2")
        self.horizontalLayout.addWidget(self.label_2)
        self.spinBox = QtWidgets.QSpinBox(self.horizontalLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.spinBox.sizePolicy().hasHeightForWidth())
        self.spinBox.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setPointSize(15)
        self.spinBox.setFont(font)
        self.spinBox.setStyleSheet("background-color: rgb(40,30,55);")
        self.spinBox.setAlignment(QtCore.Qt.AlignCenter)
        self.spinBox.setObjectName("spinBox")
        self.horizontalLayout.addWidget(self.spinBox)
        self.label_7 = QtWidgets.QLabel(self.centralwidget)
        self.label_7.setGeometry(QtCore.QRect(440, 130, 301, 61))
        font = QtGui.QFont()
        font.setPointSize(24)
        font.setBold(True)
        font.setWeight(75)
        self.label_7.setFont(font)
        self.label_7.setAlignment(QtCore.Qt.AlignCenter)
        self.label_7.setObjectName("label_7")
        self.horizontalLayoutWidget_2 = QtWidgets.QWidget(self.centralwidget)
        self.horizontalLayoutWidget_2.setGeometry(QtCore.QRect(30, 610, 391, 51))
        self.horizontalLayoutWidget_2.setObjectName("horizontalLayoutWidget_2")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget_2)
        self.horizontalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.arm_button = QtWidgets.QPushButton(self.horizontalLayoutWidget_2)
        self.arm_button.setObjectName("arm_button")
        self.horizontalLayout_2.addWidget(self.arm_button)
        self.takeoff_button = QtWidgets.QPushButton(self.horizontalLayoutWidget_2)
        self.takeoff_button.setObjectName("takeoff_button")
        self.horizontalLayout_2.addWidget(self.takeoff_button)
        self.pushButton_10 = QtWidgets.QPushButton(self.horizontalLayoutWidget_2)
        self.pushButton_10.setObjectName("pushButton_10")
        self.horizontalLayout_2.addWidget(self.pushButton_10)
        self.horizontalLayoutWidget_3 = QtWidgets.QWidget(self.centralwidget)
        self.horizontalLayoutWidget_3.setGeometry(QtCore.QRect(780, 580, 471, 87))
        self.horizontalLayoutWidget_3.setObjectName("horizontalLayoutWidget_3")
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget_3)
        self.horizontalLayout_4.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.label_8 = QtWidgets.QLabel(self.horizontalLayoutWidget_3)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_8.sizePolicy().hasHeightForWidth())
        self.label_8.setSizePolicy(sizePolicy)
        self.label_8.setMinimumSize(QtCore.QSize(150, 0))
        self.label_8.setBaseSize(QtCore.QSize(0, 0))
        font = QtGui.QFont()
        font.setPointSize(15)
        font.setBold(True)
        font.setWeight(75)
        self.label_8.setFont(font)
        self.label_8.setAlignment(QtCore.Qt.AlignCenter)
        self.label_8.setIndent(10)
        self.label_8.setObjectName("label_8")
        self.horizontalLayout_4.addWidget(self.label_8)
        self.textBrowser_2 = QtWidgets.QTextBrowser(self.horizontalLayoutWidget_3)
        self.textBrowser_2.setStyleSheet("background-color: rgb(40,30,55);")
        self.textBrowser_2.setObjectName("textBrowser_2")
        self.horizontalLayout_4.addWidget(self.textBrowser_2)
        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

        self.connect_my_signals()
        self.ros_init()

    def connect_my_signals(self):
        self.pushButton.clicked.connect(self.go_button_clicked)
        self.arm_button.clicked.connect(self.arm_button_clicked)
        self.takeoff_button.clicked.connect(self.takeoff_button_clicked)

    def takeoff_button_clicked(self):
        altitude = self.down_lineEdit.text() or "2"
        if not altitude.lstrip('-').isdigit():
            self.error_popup()
        else:
            self.ros_send_takeoff(float(altitude))
        self.down_lineEdit.clear()

    def ros_send_takeoff(self, altitude=2.0):
        self.node.get_logger().info("Sending TAKE-OFF action goal")
        self.takeoff_button.setStyleSheet("background-color : yellow")
        goal_msg = Takeoff.Goal()
        goal_msg.altitude = altitude
        while not self.node.takeoff_action_client.wait_for_server():
            self.get_logger().info('waiting for TAKE-OFF server...')

        self.takeoff_future = self.node.takeoff_action_client.send_goal_async(goal_msg)
        self.takeoff_future.add_done_callback(self.takeoff_response_callback)

    def takeoff_response_callback(self, future):
        goal_handle = future.result()
        self.node.get_logger().info('waiting for takeoff response...')
        self.takeoff_get_result_future = goal_handle.get_result_async()
        self.takeoff_get_result_future.add_done_callback(self.takeoff_get_result_callback)

    def takeoff_get_result_callback(self, future):
        result = future.result().result.result
        print(result)
        if result == 1:
            self.takeoff_button.setStyleSheet("background-color : green")
            self.node.get_logger().info('Takeoff request succesfull')
        else:
            self.takeoff_button.setStyleSheet("background-color : red")
            self.node.get_logger().info('Takeoff request error')

    def arm_button_clicked(self):
        self.node.get_logger().info("Sending ARM action goal")
        self.arm_button.setStyleSheet("background-color : yellow")

        goal_msg = Arm.Goal()

        while not self.node.arm_action_client.wait_for_server():
            self.get_logger().info('waiting for ARM server...')

        self.arm_future = self.node.arm_action_client.send_goal_async(goal_msg)
        self.arm_future.add_done_callback(self.arm_response_callback)

    def arm_response_callback(self, future):
        goal_handle = future.result()
        self.node.get_logger().info('waiting for arm response...')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.arm_get_result_callback)

    def arm_get_result_callback(self, future):
        result = future.result().result.result
        print(result)
        if result == 1:
            self.arm_button.setStyleSheet("background-color : green")
            self.node.get_logger().info('Arm request succesfull')
        else:
            self.arm_button.setStyleSheet("background-color : red")
            self.node.get_logger().info('Arm request error')

    def go_button_clicked(self):
        north = self.north_lineEdit.text() or "0"
        east = self.east_lineEdit.text() or "0"
        down = self.down_lineEdit.text() or "0"

        if not north.lstrip('-').isdigit() or not east.lstrip('-').isdigit() or not down.lstrip('-').isdigit():
            self.error_popup()
        else:
            self.ros_send_goto_relative(float(north), float(east), float(down))
        self.east_lineEdit.clear()
        self.down_lineEdit.clear()
        self.north_lineEdit.clear()

    def error_popup(self):
        msg = QMessageBox()
        msg.setWindowTitle("Invalid coordinates value")
        msg.setText("Enter valid coordinates values")
        msg.setIcon(QMessageBox.Warning)
        msg.setStandardButtons(QMessageBox.Ok)
        msg.setDefaultButton(QMessageBox.Ok)

        x = msg.exec_()

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.label_4.setText(_translate("MainWindow", "KNR Drone control"))
        self.label_5.setText(_translate("MainWindow", "Go to position"))
        self.textBrowser.setHtml(_translate("MainWindow",
                                            "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
                                            "<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
                                            "p, li { white-space: pre-wrap; }\n"
                                            "</style></head><body style=\" font-family:\'Ubuntu\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
                                            "<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p>\n"
                                            "<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p></body></html>"))
        self.pushButton_8.setText(_translate("MainWindow", " UP"))
        self.pushButton_7.setText(_translate("MainWindow", " DOWN"))
        self.pushButton.setText(_translate("MainWindow", "GO"))
        self.label_3.setText(_translate("MainWindow", "Down (z)"))
        self.label_6.setText(_translate("MainWindow", "North (x)"))
        self.label.setText(_translate("MainWindow", "East (y)"))
        self.label_2.setText(_translate("MainWindow", "Step"))
        self.label_7.setText(_translate("MainWindow", "Move"))
        self.arm_button.setText(_translate("MainWindow", "Arm"))
        self.takeoff_button.setText(_translate("MainWindow", "Take off"))
        self.pushButton_10.setText(_translate("MainWindow", "Land"))
        self.label_8.setText(_translate("MainWindow", "position"))


def main():
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
