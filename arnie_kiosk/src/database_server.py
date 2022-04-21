#!/usr/bin/env python3
""" Arnie Database ROS Service Server

A central class manages the PostgreSQL database connection, and serves insert and fetch queries
as ROS services for other nodes.
"""
import sys
import psycopg2
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
import pickle
import datetime
import std_msgs.msg
from arnie_kiosk.srv import (
    InsertUser, InsertUserResponse,
    InsertOrder, InsertOrderResponse,
    FetchUser, FetchUserResponse,
    FetchIds, FetchIdsResponse,
    FetchItemIds, FetchItemIdsResponse,
    FetchItem, FetchItemResponse
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
    order_id SERIAL NOT NULL PRIMARY KEY,
    order_time TIMESTAMP NOT NULL,
    user_id INT NOT NULL,
    item_id INT NOT NULL
);"""


class ArnieDatabaseServer(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.conn = psycopg2.connect(user="jon",
                                     password="jon",
                                     database="postgres")
        self.bridge = CvBridge()
        
        #create tables if not exist
        cursor = self.conn.cursor()
        cursor.execute(SQL_CREATE_USERS_TABLE)
        self.conn.commit()
        cursor.execute(SQL_CREATE_ORDERS_TABLE)
        self.conn.commit()
        cursor.close()

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

    def insert_order(self, request):
        insert_order_query = """INSERT INTO orders(order_time, user_id, item_id) VALUES(%s,%s,%s) RETURNING order_id"""
        insert_order_payload = (datetime.datetime.now(),request.user_id,request.item_id)

        cursor = self.conn.cursor()
        cursor.execute(insert_order_query, insert_order_payload)
        order_id = cursor.fetchone()[0]
        self.conn.commit()
        cursor.close()

        return InsertOrderResponse(order_id)
    
    def fetch_item_ids(self, request):
        fetch_item_ids_query = """SELECT item_id FROM menu"""

        cursor = self.conn.cursor()
        cursor.execute(fetch_item_ids_query)
        rows = cursor.fetchall()
        cursor.close()

        item_ids = [ id for row in rows for id in row ]

        return FetchItemIdsResponse(item_ids)

    def fetch_item(self, request):
        fetch_item_query = """SELECT name FROM menu WHERE item_id=%s"""
        fetch_item_payload = (request.item_id,)

        cursor = self.conn.cursor()
        cursor.execute(fetch_item_query, fetch_item_payload)
        row = cursor.fetchall()[0]
        cursor.close()

        print(row)
        name = row[0]

        return FetchItemResponse(name)


def main():
    db_server = ArnieDatabaseServer()

    rospy.init_node('database_server')
    rospy.Service('insert_user', InsertUser, db_server.insert_user)
    rospy.Service('insert_order', InsertOrder, db_server.insert_order)
    rospy.Service('fetch_user', FetchUser, db_server.fetch_user_entry)
    rospy.Service('fetch_ids', FetchIds, db_server.fetch_user_ids)
    rospy.Service('fetch_item_ids', FetchItemIds, db_server.fetch_item_ids)
    rospy.Service('fetch_item', FetchItem, db_server.fetch_item)

    rospy.spin()


if __name__ == "__main__":
    sys.exit(main())
