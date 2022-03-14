import sys
from PySide6.QtWidgets import QApplication

# from database import ArnieDatabaseManager
# from camera import ArnieCameraManager
from gui import MainWindow

PROJECT_DATABASE = "arnie.sqlite"
CAMERA_DEVICE_ID = 0

def main():
    app = QApplication()
    window = MainWindow()
    window.showMaximized()
    app.exec()


if __name__ == "__main__":
    sys.exit(main())
