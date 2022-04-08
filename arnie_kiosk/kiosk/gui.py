#!/usr/bin/env python3
import sys
from turtle import right
from PySide6.QtCore import (
    Qt,
    Slot,
    QStandardPaths,
    QObject,
    Signal
)
from PySide6.QtWidgets import (
    QApplication,
    QMainWindow,
    QHBoxLayout,
    QVBoxLayout,
    QWidget,
    QPushButton,
    QLabel,
    QStackedWidget,
    QLineEdit
)
from PySide6.QtMultimedia import (
    QCamera,
    QImageCapture,
    QMediaCaptureSession,
    QMediaDevices,

)
from PySide6.QtGui import (
    QPixmap
)
from PySide6.QtMultimediaWidgets import (
    QVideoWidget
)


class GuiSignals(QObject):
    setPage = Signal(int)
    setNewUserInfo = Signal(str,str,str)


class IdlePage(QWidget):
    """PySide6 Widget implementing the splash page for Arnie."""
    def __init__(self, gui_signals):
        super().__init__()

        self.gui_signals = gui_signals

        layout = QVBoxLayout()

        self.title_text = QLabel("ArnieBot - The Original Iced Tea + Lemonade\nBeverage Service System")
        font = self.title_text.font()
        font.setPointSize(32)
        self.title_text.setFont(font)
        self.title_text.setAlignment(Qt.AlignHCenter|Qt.AlignVCenter)

        self.prompt_text = QLabel("Touch anywhere to begin")
        font = self.prompt_text.font()
        font.setPointSize(24)
        self.prompt_text.setFont(font)
        self.prompt_text.setAlignment(Qt.AlignHCenter|Qt.AlignVCenter)

        layout.addWidget(self.title_text)
        layout.addWidget(self.prompt_text)
        self.setLayout(layout)
    
    def mousePressEvent(self, event):
        print("click...")
        self.gui_signals.setPage.emit(1)



class RegistrationPage(QWidget):
    """PySide6 Widget implementing the user registration page for Arnie."""
    def __init__(self, gui_signals):
        super().__init__()

        self.gui_signals = gui_signals

        self.pictures_location = QStandardPaths.writableLocation(QStandardPaths.PicturesLocation)
        self.picture_filename = f"{self.pictures_location}/tmp.jpg"

        self.greeting_text = QLabel("Welcome new user!\nPlease enter your name\nand take a profile picture")
        font = self.greeting_text.font()
        font.setPointSize(16)
        self.greeting_text.setFont(font)
        self.greeting_text.setAlignment(Qt.AlignHCenter|Qt.AlignVCenter)

        self.first_name_label = QLabel("First Name:")
        self.first_name_label.setAlignment(Qt.AlignLeft|Qt.AlignBottom)
        self.first_name_label.setFixedHeight(32)
        self.first_name_edit = QLineEdit("Arnold")
        self.first_name_edit.setFixedWidth(240)

        self.last_name_label = QLabel("Last Name:")
        self.last_name_label.setAlignment(Qt.AlignLeft|Qt.AlignBottom)
        self.last_name_label.setFixedHeight(32)
        self.last_name_edit = QLineEdit("Palmer")
        self.last_name_edit.setFixedWidth(240)

        self.spacer_label = QLabel("")
    
        self.take_picture_button = QPushButton("Take Picture")
    
        self.cancel_button = QPushButton("Cancel")

        self.left_layout = QVBoxLayout()
        self.left_layout.addWidget(self.greeting_text)
        self.left_layout.addWidget(self.first_name_label)
        self.left_layout.addWidget(self.first_name_edit)
        self.left_layout.addWidget(self.last_name_label)
        self.left_layout.addWidget(self.last_name_edit)
        self.left_layout.addWidget(self.spacer_label)
        self.left_layout.addWidget(self.take_picture_button)
        self.left_layout.addWidget(self.cancel_button)

        self.viewfinder = QVideoWidget()

        available_cameras = QMediaDevices.videoInputs()
        if available_cameras:
            self._camera_info = available_cameras[0]

            self._camera = QCamera(self._camera_info)
            self._camera.errorOccurred.connect(self._camera_error)

            self._image_capture = QImageCapture(self._camera)
            # self._image_capture.imageCaptured.connect(self.image_captured)
            self._image_capture.imageSaved.connect(self.image_saved)
            self._image_capture.errorOccurred.connect(self._capture_error)

            self._capture_session = QMediaCaptureSession()
            self._capture_session.setCamera(self._camera)
            self._capture_session.setImageCapture(self._image_capture)

        main_layout = QHBoxLayout()
        main_layout.addLayout(self.left_layout)
        main_layout.addWidget(self.viewfinder)
        self.setLayout(main_layout)

        self.take_picture_button.clicked.connect(self.take_picture)
        self.cancel_button.clicked.connect(self.cancel)

        if self._camera and self._camera.error() == QCamera.NoError:
            # name = self._camera_info.description()
            # self.setWindowTitle(f"PySide6 Camera Example ({name})")
            # self.show_status_message(f"Starting: '{name}'")

            self._capture_session.setVideoOutput(self.viewfinder)
            self._camera.start()
        else:
            self.setWindowTitle("PySide6 Camera Example")
            self.show_status_message("Camera unavailable")
    
    @Slot()
    def take_picture(self):
        print("taking picture...")
        self._image_capture.captureToFile(self.picture_filename)

    @Slot(int, str)
    def image_saved(self, id, fileName):
        first_name = self.first_name_edit.text()
        last_name = self.last_name_edit.text()
        self.gui_signals.setNewUserInfo.emit(first_name, last_name, self.picture_filename)
        self.gui_signals.setPage.emit(2)

    def closeEvent(self, event):
        print("closing reg page")
        if self._camera and self._camera.isActive():
            self._camera.stop()

    @Slot(int, QImageCapture.Error, str)
    def _capture_error(self, id, error, error_string):
        print(error_string, file=sys.stderr)

    @Slot(QCamera.Error, str)
    def _camera_error(self, error, error_string):
        print(error_string, file=sys.stderr)

    def cancel(self):
        if self._camera and self._camera.isActive():
            self._camera.stop()
        self.gui_signals.setPage.emit(0)


class ConfirmationPage(QWidget):
    def __init__(self, gui_signals):
        super().__init__()

        self.gui_signals = gui_signals

        self.gui_signals.setNewUserInfo.connect(self.update_preview)

        main_layout = QHBoxLayout()

        self._confirm_text = QLabel("Does this look OK?")
        font = self._confirm_text.font()
        font.setPointSize(18)
        self._confirm_text.setFont(font)
        self._confirm_text.setAlignment(Qt.AlignHCenter|Qt.AlignVCenter)

        self._approve_button = QPushButton("Approve")
        self._approve_button.clicked.connect(self.approve)
        self._retake_button = QPushButton("Retake")
        self._retake_button.clicked.connect(self.retake)
        self._cancel_button = QPushButton("Cancel")
        self._cancel_button.clicked.connect(self.cancel)

        left_layout = QVBoxLayout()
        left_layout.addWidget(self._confirm_text)
        left_layout.addWidget(self._approve_button)
        left_layout.addWidget(self._retake_button)
        left_layout.addWidget(self._cancel_button)

        main_layout.addLayout(left_layout)

        self._profile_picture = QLabel()
        pictures_location = QStandardPaths.writableLocation(QStandardPaths.PicturesLocation)
        self._profile_picture.setPixmap(QPixmap(f"{pictures_location}/tmp.jpg")) #TODO: init better
        
        self._first_name = "NOINIT_FIRST"
        self._last_name = "NOINIT_LAST"
        self._profile_name = QLabel(f"Name: '{self._first_name} {self._last_name}'")
        font = self._profile_name.font()
        font.setPointSize(18)
        self._profile_name.setFont(font)
        self._profile_name.setAlignment(Qt.AlignHCenter | Qt.AlignTop)
        self._profile_name.setFixedHeight(100)

        right_layout = QVBoxLayout()

        right_layout.addWidget(self._profile_picture)
        right_layout.addWidget(self._profile_name)

        main_layout.addLayout(right_layout)
        self.setLayout(main_layout)

    @Slot(str,str,str)
    def update_preview(self, first_name, last_name, picture_filename):
        self._first_name = first_name
        self._last_name = last_name
        self._profile_picture.setPixmap(QPixmap(picture_filename))

        self._confirm_text.setText(f"Thanks {self._first_name}!\nDoes this look OK?")
        self._profile_name.setText(f"Name: '{self._first_name} {self._last_name}'")
    
    def approve(self):
        print("Approved!")

    def retake(self):
        self.gui_signals.setPage.emit(1)
    
    def cancel(self):
        self.gui_signals.setPage.emit(0)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Arnie Kiosk")

        self.gui_signals = GuiSignals()
        self.gui_signals.setPage.connect(self.set_page)

        self._stacked_widget = QStackedWidget()
        self.setCentralWidget(self._stacked_widget)

        self._idle_page = IdlePage(self.gui_signals)
        self._stacked_widget.addWidget(self._idle_page)

        self._registration_page = RegistrationPage(self.gui_signals)
        self._stacked_widget.addWidget(self._registration_page)

        self._confirmation_page = ConfirmationPage(self.gui_signals)
        self._stacked_widget.addWidget(self._confirmation_page)


    def closeEvent(self, event):
        print('closing MainWindow')
        self._registration_page.closeEvent(event)

    @Slot(int)
    def set_page(self, index):
        print(f"trying to switch to index {index}")
        self._confirmation_page.update_preview
        self._stacked_widget.setCurrentIndex(index)


if __name__ == "__main__":
    app = QApplication()
    window = MainWindow()
    window.showMaximized()
    sys.exit(app.exec())
