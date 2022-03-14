#!/usr/bin/env python3
import sys
from turtle import right
from PySide6.QtCore import (
    Qt,
    Slot,
    QStandardPaths,
    QObject,
    Signal,
    QThread
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
from PySide6.QtGui import (
    QImage,
    QPixmap
)
from PySide6.QtMultimediaWidgets import (
    QVideoWidget
)
import cv2


PICTURE_VIEW_WIDTH = 320
PICTURE_VIEW_HEIGHT = 240
TITLE_FONT_SIZE = 16
BODY_FONT_SIZE = 12
BUTTON_HEIGHT = 48

class GuiSignals(QObject):
    setPage = Signal(int)
    setNewUserInfo = Signal(str,str,str)


class UserEntry(object):
    def __init__(self, first_name, last_name, profile_picture):
        self.first_name = first_name
        self.last_name = last_name
        self.profile_picture = profile_picture



class CameraThread(QThread):
    updateFrame = Signal(QImage)

    def __init__(self):
        QThread.__init__(self)

    def run(self):
        self.cap = cv2.VideoCapture(0)
        self.keep_alive = True

        while self.keep_alive:
            ret, frame = self.cap.read()
            if not ret:
                continue

            #transform frame
            frame = cv2.flip(frame, 1)

            #convert frame to img
            h, w, ch = frame.shape
            img = QImage(frame.data, w, h, ch*w, QImage.Format_RGB888)

            #size down img
            img = img.scaled(PICTURE_VIEW_WIDTH, PICTURE_VIEW_HEIGHT, Qt.KeepAspectRatio)

            #emit img
            self.updateFrame.emit(img)

        self.cap.release()
        cv2.destroyAllWindows()

    def quit(self):
        self.keep_alive = False


class IdlePage(QWidget):
    """PySide6 Widget implementing the splash page for Arnie."""
    leaveIdle = Signal()

    def __init__(self, enterPageSignal):
        super().__init__()

        enterPageSignal.connect(self.enter_page)

        layout = QVBoxLayout()

        self.title_text = QLabel("ArnieBot - The Original Iced Tea + Lemonade\nBeverage Service System")
        font = self.title_text.font()
        font.setPointSize(TITLE_FONT_SIZE)
        self.title_text.setFont(font)
        self.title_text.setAlignment(Qt.AlignHCenter|Qt.AlignVCenter)

        self.prompt_text = QLabel("Touch anywhere to begin")
        font = self.prompt_text.font()
        font.setPointSize(BODY_FONT_SIZE)
        self.prompt_text.setFont(font)
        self.prompt_text.setAlignment(Qt.AlignHCenter|Qt.AlignVCenter)

        layout.addWidget(self.title_text)
        layout.addWidget(self.prompt_text)
        self.setLayout(layout)
    
    def enter_page(self):
        print('entering idle page')

    def leave_page(self):
        print('leaving idle page')

    def mousePressEvent(self, event):
        print("click...")
        self.leave_page()
        self.leaveIdle.emit()


class RegistrationPage(QWidget):
    """PySide6 Widget implementing the user registration page for Arnie."""
    
    cancelRegistration = Signal()
    setRegistrationDetails = Signal(str,str,QImage)

    def __init__(self, enterPageSignal):
        super().__init__()

        enterPageSignal.connect(self.enter_registration)

        self.pictures_location = QStandardPaths.writableLocation(QStandardPaths.PicturesLocation)
        self.picture_filename = f"{self.pictures_location}/tmp.jpg"

        self.greeting_text = QLabel("Welcome new user!\nPlease enter your name\nand take a profile picture")
        font = self.greeting_text.font()
        font.setPointSize(BODY_FONT_SIZE)
        self.greeting_text.setFont(font)
        self.greeting_text.setAlignment(Qt.AlignHCenter|Qt.AlignVCenter)

        self.first_name_label = QLabel("First Name:")
        self.first_name_label.setAlignment(Qt.AlignLeft|Qt.AlignBottom)
        self.first_name_label.setFixedHeight(24)
        self.first_name_edit = QLineEdit("Arnold")
        self.first_name_edit.setFixedWidth(180)

        self.last_name_label = QLabel("Last Name:")
        self.last_name_label.setAlignment(Qt.AlignLeft|Qt.AlignBottom)
        self.last_name_label.setFixedHeight(24)
        self.last_name_edit = QLineEdit("Palmer")
        self.last_name_edit.setFixedWidth(180)

        self.spacer_label = QLabel("")
    
        self.take_picture_button = QPushButton("Take Picture")
        self.take_picture_button.setFixedHeight(BUTTON_HEIGHT)
    
        self.cancel_button = QPushButton("Cancel")
        self.cancel_button.setFixedHeight(BUTTON_HEIGHT)

        self.left_layout = QVBoxLayout()
        self.left_layout.addWidget(self.greeting_text)
        self.left_layout.addWidget(self.first_name_label)
        self.left_layout.addWidget(self.first_name_edit)
        self.left_layout.addWidget(self.last_name_label)
        self.left_layout.addWidget(self.last_name_edit)
        self.left_layout.addWidget(self.spacer_label)
        self.left_layout.addWidget(self.take_picture_button)
        self.left_layout.addWidget(self.cancel_button)

        self.camera_view_label = QLabel(self)
        self.camera_view_label.setFixedSize(PICTURE_VIEW_WIDTH, PICTURE_VIEW_HEIGHT)

        self.camera_view_img = QImage()

        self.camera_thread = CameraThread()
        self.camera_thread.updateFrame.connect(self.set_image)

        main_layout = QHBoxLayout()
        main_layout.addLayout(self.left_layout)
        main_layout.addWidget(self.camera_view_label)
        self.setLayout(main_layout)

        self.take_picture_button.clicked.connect(self.take_picture)
        self.cancel_button.clicked.connect(self.cancel)

    def enter_registration(self):
        print("entering reg page")
        self.camera_view_label.clear()
        self.camera_thread.start()

    def leave_registration(self):
        print("leaving reg page")
        self.camera_thread.quit()

    @Slot(QImage)
    def set_image(self, image):
        self.camera_view_img = image
        self.camera_view_label.setPixmap(QPixmap.fromImage(image))
    
    @Slot()
    def take_picture(self):
        print("taking picture...")
        first_name = self.first_name_edit.text()
        last_name = self.last_name_edit.text()
        
        self.leave_registration()
        self.setRegistrationDetails.emit(first_name, last_name, self.camera_view_img)

    def cancel(self):
        self.leave_registration()
        self.cancelRegistration.emit()


class ConfirmationPage(QWidget):

    cancelRegistration = Signal()
    retakeProfile = Signal()
    confirmUserInfo = Signal()

    def __init__(self, enterPageSignal):
        super().__init__()

        enterPageSignal.connect(self.enter_confirmation)

        main_layout = QHBoxLayout()

        self._confirm_text = QLabel("Does this look OK?")
        font = self._confirm_text.font()
        font.setPointSize(BODY_FONT_SIZE)
        self._confirm_text.setFont(font)
        self._confirm_text.setAlignment(Qt.AlignHCenter|Qt.AlignVCenter)

        self._approve_button = QPushButton("Approve")
        self._approve_button.setFixedHeight(BUTTON_HEIGHT)
        self._approve_button.clicked.connect(self.approve)
        self._retake_button = QPushButton("Retake")
        self._retake_button.setFixedHeight(BUTTON_HEIGHT)
        self._retake_button.clicked.connect(self.retake)
        self._cancel_button = QPushButton("Cancel")
        self._cancel_button.setFixedHeight(BUTTON_HEIGHT)
        self._cancel_button.clicked.connect(self.cancel)

        left_layout = QVBoxLayout()
        left_layout.addWidget(self._confirm_text)
        left_layout.addWidget(self._approve_button)
        left_layout.addWidget(self._retake_button)
        left_layout.addWidget(self._cancel_button)

        main_layout.addLayout(left_layout)

        self._profile_picture = QLabel(self)
        self._profile_picture.setFixedSize(PICTURE_VIEW_WIDTH, PICTURE_VIEW_HEIGHT)
        
        self._first_name = "NOINIT_FIRST"
        self._last_name = "NOINIT_LAST"
        self._profile_name = QLabel(f"Name: '{self._first_name} {self._last_name}'")
        font = self._profile_name.font()
        font.setPointSize(BODY_FONT_SIZE)
        self._profile_name.setFont(font)
        self._profile_name.setAlignment(Qt.AlignHCenter | Qt.AlignTop)
        self._profile_name.setFixedHeight(32)

        right_layout = QVBoxLayout()

        right_layout.addWidget(self._profile_picture)
        right_layout.addWidget(self._profile_name)

        main_layout.addLayout(right_layout)
        self.setLayout(main_layout)

    @Slot(str,str,QImage)
    def enter_confirmation(self, first_name, last_name, profile_picture):
        print("entering conf page")
        self._profile_picture.clear()

        self._first_name = first_name
        self._last_name = last_name
        self._profile_name.setText(f"Name: '{self._first_name} {self._last_name}'")

        self._profile_picture.setPixmap(QPixmap.fromImage(profile_picture))

    def leave_confirmation(self):
        print("leaving conf page")

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
        self.retakeProfile.emit()

    def cancel(self):
        self.cancelRegistration.emit()


class MainWindow(QMainWindow):
    enterIdle = Signal()
    enterRegistration = Signal()
    enterConfirmation = Signal(str,str,QImage)

    def __init__(self):
        super().__init__()

        self.setWindowTitle("Arnie Kiosk")

        self._stacked_widget = QStackedWidget()
        self.setCentralWidget(self._stacked_widget)

        self._idle_page = IdlePage(self.enterIdle)
        self._stacked_widget.addWidget(self._idle_page)
        self._idle_page.leaveIdle.connect(self.leave_idle)

        self._registration_page = RegistrationPage(self.enterRegistration)
        self._stacked_widget.addWidget(self._registration_page)
        self._registration_page.cancelRegistration.connect(self.cancel_registration)
        self._registration_page.setRegistrationDetails.connect(self.intake_user_reg)

        self._confirmation_page = ConfirmationPage(self.enterConfirmation)
        self._stacked_widget.addWidget(self._confirmation_page)
        self._confirmation_page.cancelRegistration.connect(self.cancel_registration)
        self._confirmation_page.retakeProfile.connect(self.retake_user_info)

    def leave_idle(self):
        #TODO: add logic here to select based on recoged/unrecoged face!
        #TODO: need better scheme for indexing pages in __init__
        self._stacked_widget.setCurrentIndex(1) #hardcoding reg page index
        self.enterRegistration.emit()
    
    def cancel_registration(self):
        self._stacked_widget.setCurrentIndex(0) #TODO: fix widget hardcode
        self.enterIdle.emit()

    @Slot(str,str,QImage)
    def intake_user_reg(self, first_name, last_name, profile_picture):
        self._stacked_widget.setCurrentIndex(2)
        self.enterConfirmation.emit(first_name, last_name, profile_picture)

    def retake_user_info(self):
        self._stacked_widget.setCurrentIndex(1)
        self.enterRegistration.emit()


if __name__ == "__main__":
    app = QApplication()
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
