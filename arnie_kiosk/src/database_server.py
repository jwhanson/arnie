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
import std_msgs.msg
from arnie_kiosk.srv import (
    InsertUser, InsertUserResponse,
    FetchUser, FetchUserResponse,
    FetchIds, FetchIdsResponse
)


SQL_CREATE_USERS_TABLE = """
CREATE TABLE IF NOT EXISTS users (
    user_id SERIAL PRIMARY KEY,
    first_name VARCHAR (16) NOT NULL,
    last_name VARCHAR (16) NOT NULL,
    profile_picture BYTEA NOT NULL
);"""

SQL_CREATE_ORDERS_TABLE = """
CREATE TABLE IF NOT EXISTS orders (
    order_id SERIAL PRIMARY KEY,
    timestamp DATETIME,
    user_id INTEGER,
    menu_id INTEGER
);"""


class ArnieDatabaseServer(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.conn = psycopg2.connect(user="jon",
                                     password="jon",
                                     database="postgres")
        self.bridge = CvBridge()
        self.create_users_table()

    def create_users_table(self):
        cursor = self.conn.cursor()
        cursor.execute(SQL_CREATE_USERS_TABLE)
        self.conn.commit()

    def fetch_user_ids(self, request):
        fetch_user_ids_query = """SELECT user_id FROM users"""

        cursor = self.conn.cursor()
        cursor.execute(fetch_user_ids_query)
        rows = cursor.fetchall()
        cursor.close()

        user_ids = [ id for row in rows for id in row ]

        return FetchIdsResponse(user_ids)

    def insert_user(self, request):
        profile_picture = self.bridge.imgmsg_to_cv2(request.profile_picture)
        profile_picture_pickle = pickle.dumps(profile_picture)
        insert_user_query = """INSERT INTO users(first_name,last_name,profile_picture) VALUES(%s,%s,%s) RETURNING user_id"""
        insert_user_payload = (request.first_name, request.last_name, profile_picture_pickle)

        cursor = self.conn.cursor()
        cursor.execute(insert_user_query, insert_user_payload)
        user_id = cursor.fetchone()[0]
        self.conn.commit()
        cursor.close()
        
        return InsertUserResponse(user_id)

    def fetch_user_entry(self, request):
        fetch_user_entry_query = """SELECT * FROM users WHERE user_id=%s"""
        fetch_user_entry_payload = (request.id,)

        cursor = self.conn.cursor()
        cursor.execute(fetch_user_entry_query, fetch_user_entry_payload)
        row = cursor.fetchall()[0]
        cursor.close()

        first_name = std_msgs.msg.String(row[1])
        last_name = std_msgs.msg.String(row[2])

        profile_picture_pickle = bytes(row[3])
        profile_picture = pickle.loads(profile_picture_pickle)
        profile_picture_msg = self.bridge.cv2_to_imgmsg(profile_picture)

        return FetchUserResponse(first_name, last_name, profile_picture_msg)

    def check_user(self, user_id):
        check_query = """SELECT * FROM users WHERE user_id=%s"""
        check_payload = (user_id,)

        cursor = self.conn.cursor()
        cursor.execute(check_query, check_payload)

        results = cursor.fetchall()
        cursor.close()

        user = results[0]

        print(user)
        first_name = user[1]
        last_name = user[2]
        profile_picture = pickle.loads(user[3])

        # Nice Debug Prints:
        print(first_name)
        print(last_name)
        cv2.imshow("insert", profile_picture)
        cv2.waitKey()

        return


def main():
    db_server = ArnieDatabaseServer()

    rospy.init_node('database_server')
    rospy.Service('insert_user', InsertUser, db_server.insert_user)
    rospy.Service('fetch_user', FetchUser, db_server.fetch_user_entry)
    rospy.Service('fetch_ids', FetchIds, db_server.fetch_user_ids)

    rospy.spin()


if __name__ == "__main__":
    sys.exit(main())
