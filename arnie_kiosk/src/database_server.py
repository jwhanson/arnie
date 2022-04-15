#!/usr/bin/env python3
""" Arnie Database ROS Service Server

following: https://pynative.com/python-postgresql-tutorial/#h-perform-postgresql-crud-operations-from-python
"""
import sys
import psycopg2
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
import pickle
from arnie_kiosk.srv import InsertUser


SQL_CREATE_USERS_TABLE = """
CREATE TABLE IF NOT EXISTS users (
    user_id INTEGER PRIMARY KEY,
    name TEXT,
    picture BLOB
);"""

SQL_CREATE_ORDERS_TABLE = """
CREATE TABLE IF NOT EXISTS orders (
    order_id integer PRIMARY KEY,
    timestamp DATETIME,
    user_id INTEGER,
    menu_id INTEGER
);"""

SQL_CREATE_MENU_TABLE = """
CREATE TABLE IF NOT EXISTS menu (
    item_id INTEGER PRIMARY KEY,
    name TEXT,
    ingredients TEXT
);"""


class ArnieDatabaseServer(object):
    def __init__(self):
        self.bridge = CvBridge()

    def insert_user(self, request):
        profile_picture = self.bridge.imgmsg_to_cv2(request.profile_picture)
        profile_picture_pickle = pickle.dumps(profile_picture)
        print(request.first_name)
        print(request.last_name)
        cv2.imshow("insert", profile_picture)
        cv2.waitKey()
        print("""INSERT INTO users(first_name last_name img_pickle) VALUES(%s %s %s)""")
        return 0


def main():
    db_server = ArnieDatabaseServer()

    rospy.init_node('database_server')
    s = rospy.Service('insert_user', InsertUser, db_server.insert_user)

    rospy.spin()


if __name__ == "__main__":
    sys.exit(main())
