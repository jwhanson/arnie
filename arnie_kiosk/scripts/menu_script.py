import psycopg2
import toml
import sys

SQL_CREATE_MENU_TABLE = """
CREATE TABLE IF NOT EXISTS menu (
    item_id SERIAL PRIMARY KEY,
    name TEXT,
    ingredients TEXT
);"""

conn = psycopg2.connect(user="jon",
                        password="jon",
                        database="postgres")

cursor = conn.cursor()
cursor.execute(SQL_CREATE_MENU_TABLE)
conn.commit()

check_query = """SELECT * FROM menu;"""
cursor.execute(check_query)
response = cursor.fetchall()

if len(response) > 0:
    print("table already exists; doing nothing")
    sys.exit()

print("populating menu table from menu.toml")
menu = toml.load("menu.toml")
for item in menu["items"]:
    insert_item_query = """INSERT INTO menu(name,ingredients) VALUES(%s,%s)"""
    insert_item_payload = (item["name"],item["ingredients"])
    cursor.execute(insert_item_query, insert_item_payload)
    conn.commit()
