'''Arnie Database Management Module
Jonathan Hanson

IMPORTANT: This module is no longer intended for use in the final project.
SQL functionality is unimplemented in the System-Integration build. In the
future, a ROS node is expected to own SQL interactions, streamlining how
we save the database and how it is exposed to relevant nodes.

This module owns the sqlite3 database connection, helping to simplify how
other modules get information into and out of the database.
'''
import sqlite3

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

ARNIE_TABLE_SQL_COMMANDS = [
    SQL_CREATE_USERS_TABLE,
    SQL_CREATE_ORDERS_TABLE,
    SQL_CREATE_MENU_TABLE
]

class ArnieDatabaseManager(object):
    def __init__(self, db_file):
        self.conn = None

        try:
            self.conn = sqlite3.connect(db_file)
        except sqlite3.Error as e:
            print(e)

        for create_table_sql in ARNIE_TABLE_SQL_COMMANDS:
            try:
                c = self.conn.cursor()
                c.execute(create_table_sql)
            except sqlite3.Error as e:
                print(e)

        return


    def create_user(self, name, img_blob):
        """Insert a new user into the users table

        Params:
            conn: SQLite DB connection object.

        Returns:
            None
        """
        sql_cmd = """INSERT INTO users(name, picture) VALUES(?, ?)"""

        cur = self.conn.cursor()
        cur.execute( sql_cmd, (name, sqlite3.Binary(img_blob)) )
        self.conn.commit()

        return

    def query_user(self, user_id=None, user_name=None):
        """Query the DB for a user_id and open the associated profile picture with OpenCV.

        Params:
            user_id: User ID to be queried from the database.

        Returns:
            (tuple) number of matching entries, and list of sqlite row entries.
        """
        if user_id == None and user_name == None:
            print("No user info given.")
            return

        cur = self.conn.cursor()
        if user_id != None:
            sql_cmd = """SELECT * FROM users WHERE user_id=?"""
            cur.execute(sql_cmd, (user_id,))
        elif user_name != None:
            sql_cmd = """SELECT * FROM users WHERE name=?"""
            cur.execute(sql_cmd, (user_name,))

        rows = cur.fetchall()
        if not rows:
            print(f"No users found with user_id={user_id} or name='{user_name}'")
            return

        return (len(rows), rows)
