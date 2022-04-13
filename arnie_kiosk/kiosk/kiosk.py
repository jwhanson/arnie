#!/usr/bin/env python3
'''Arnie Kiosk Top-Level Executable

IMPORTANT: This "top-level" module will not be necessary as we move direct
database and camera functionality out of the kiosk module.

This module runs the Arnie Kiosk, bringing together the Camera Mangement,
Database Management, and Kiosk GUI modules.

A simple ROS publisher is implemented to serve an order_id UInt16 msg to the
topic /order.
'''
import sys
from PySide2.QtWidgets import QApplication
import rospy
from std_msgs.msg import UInt16

# from database import ArnieDatabaseManager
# from camera import ArnieCameraManager
from gui import MainWindow

PROJECT_DATABASE = "arnie.sqlite"
CAMERA_DEVICE_ID = 0

def main():
    order_pub = rospy.Publisher("order", UInt16)

    print("Blocking until registered with ROS master...")
    rospy.init_node("kiosk")
    print("Success! Registered with ROS master")

    app = QApplication()
    window = MainWindow(order_pub)
    window.showFullScreen()
    app.exec_()


if __name__ == "__main__":
    sys.exit(main())
