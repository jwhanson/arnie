import sys

from database import ArnieDatabaseManager
from camera import ArnieCameraManager

PROJECT_DATABASE = "arnie.sqlite"
CAMERA_DEVICE_ID = 0

def take_user_name():
    """"""
    name = input("Type your prefered name: ")
    return name.strip()

def main(args):
    dbman = ArnieDatabaseManager(PROJECT_DATABASE)
    cvman = ArnieCameraManager(CAMERA_DEVICE_ID)

    name = take_user_name()
    img_blob = cvman.take_user_picture()
    dbman.create_user(name, img_blob)

    [num_rows, rows] = dbman.query_user(user_id=1)

    for row in rows:
        cvman.display_image_blob(row[1], row[2])


if __name__ == "__main__":
    sys.exit(main(sys.argv))