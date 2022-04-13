'''Arnie GUI Implementation

This module implements the Arnie Kiosk GUI with the PySide2 Qt library. The
design is a MainWindow with a central StackedWidget where all indivdual "pages"
are implemented as widgets and accessed by index from the StackedWidget.

An important design pressure is to decouple the windows as much as possible.
Evantually, perhaps a ArniePage parent class that defines the interface could
help with this.

In the latest update, the Registration Page now gets its images for video
feedback from the 'frame' ROS topic, which is published by a 'camera' ROS node
and consumed by the 'gui' and 'recognizer' nodes.
'''
import sys
from PySide2.QtCore import (
    Qt,
    Slot,
    QStandardPaths,
    QObject,
    Signal,
    QThread
)
from PySide2.QtWidgets import (
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
from PySide2.QtGui import (
    QImage,
    QPixmap
)
import cv2
import rospy
import std_msgs.msg
import sensor_msgs.msg
from cv_bridge import CvBridge


PICTURE_VIEW_WIDTH = 320
PICTURE_VIEW_HEIGHT = 240
TITLE_FONT_SIZE = 16
BODY_FONT_SIZE = 12
BUTTON_HEIGHT = 48


class RosThread(QThread):
    def __init__(self):
        super().__init__()
    
    def run(self):
        rospy.spin()


class IdlePage(QWidget):
    """PySide Widget implementing the splash page for Arnie."""
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
    """PySide2 Widget implementing the user registration page for Arnie."""
    
    cancelRegistration = Signal()
    setRegistrationDetails = Signal(str,str,QImage)

    def __init__(self, enterPageSignal, updateFrameSignal):
        super().__init__()

        enterPageSignal.connect(self.enter_registration)
        updateFrameSignal.connect(self.set_image)

        #NOTE: getting a path we can actually write with is kinda funny:
        # QStandardPaths.PicturesLocation is a object that "knows" where 
        # Pictures is, but doesn't return a path string like: 
        # "/home/jon/Pictures". QStandardPaths.writableLocation returns the
        # path string we need for a filename!
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

        main_layout = QHBoxLayout()
        main_layout.addLayout(self.left_layout)
        main_layout.addWidget(self.camera_view_label)
        self.setLayout(main_layout)

        self.take_picture_button.clicked.connect(self.take_picture)
        self.cancel_button.clicked.connect(self.cancel)

    def enter_registration(self):
        print("entering reg page")
        self.camera_view_label.clear()

    def leave_registration(self):
        print("leaving reg page")

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

    cancelConfirmation = Signal()
    retakeProfile = Signal()
    confirmUserInfo = Signal()

    def __init__(self, enterPageSignal):
        super().__init__()

        enterPageSignal.connect(self.enter)

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
    def enter(self, first_name, last_name, profile_picture):
        print("entering conf page")
        self._profile_picture.clear()

        self._first_name = first_name
        self._last_name = last_name
        self._profile_name.setText(f"Name: '{self._first_name} {self._last_name}'")

        self._profile_picture.setPixmap(QPixmap.fromImage(profile_picture))

    def leave(self):
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
        self.leave()
        self.confirmUserInfo.emit()

    def retake(self):
        self.leave()
        self.retakeProfile.emit()

    def cancel(self):
        self.leave()
        self.cancelRegistration.emit()


class OrderPage(QWidget):
    '''PySide widget implementing a simple demo ordering system.'''

    cancelOrder = Signal()
    dispatchOrder = Signal(int)

    #TODO: we need to formalize the concept of a menu in the program
    _menu_text = """Please enter an order ID:
1 - Classic Arnold Palmer
2 - Just Lemonade
3 - A Bit of Everything"""

    def __init__(self, enterPageSignal):
        super().__init__()

        enterPageSignal.connect(self.enter)

        main_layout = QVBoxLayout()

        self._order_text = QLabel(self._menu_text)
        font = self._order_text.font()
        font.setPointSize(BODY_FONT_SIZE)
        self._order_text.setFont(font)
        self._order_text.setAlignment(Qt.AlignHCenter|Qt.AlignVCenter)

        main_layout.addWidget(self._order_text)

        self._order_id_edit = QLineEdit()
        self._order_id_edit.setFixedWidth(240) #TODO: unhardcode

        self._submit_button = QPushButton("Submit")
        self._submit_button.setFixedHeight(BUTTON_HEIGHT)
        self._submit_button.clicked.connect(self.submit)

        self._cancel_button = QPushButton("Cancel")
        self._cancel_button.setFixedHeight(BUTTON_HEIGHT)
        self._cancel_button.clicked.connect(self.cancel)

        bot_layout = QHBoxLayout()
        bot_layout.addWidget(self._order_id_edit)

        bot_right_sublayout = QVBoxLayout()
        bot_right_sublayout.addWidget(self._submit_button)
        bot_right_sublayout.addWidget(self._cancel_button)

        bot_layout.addLayout(bot_right_sublayout)
        main_layout.addLayout(bot_layout)
        self.setLayout(main_layout)

    def enter(self):
        print("entering order page")

    def leave(self):
        print("leaving order page")
    
    def submit(self):
        order_id = int(self._order_id_edit.text())
        self.dispatchOrder.emit(order_id)

    def cancel(self):
        self.leave()
        self.cancelOrder.emit()


class MainWindow(QMainWindow):
    current_page_index = 0

    updateFrame = Signal(QImage)

    enterIdle = Signal()
    enterRegistration = Signal()
    enterConfirmation = Signal(str,str,QImage)
    enterOrder = Signal()

    def __init__(self, ros_order_pub):
        super().__init__()
        self.bridge = CvBridge()
        self.ros_order_pub = ros_order_pub
        self.ros_thread = RosThread()
        self.ros_thread.start()

        self.setWindowTitle("Arnie Kiosk")

        self._stacked_widget = QStackedWidget()
        self.setCentralWidget(self._stacked_widget)

        self._idle_page = IdlePage(self.enterIdle)
        self._stacked_widget.addWidget(self._idle_page)
        self._idle_page.leaveIdle.connect(self.leave_idle)

        self._registration_page = RegistrationPage(self.enterRegistration, self.updateFrame)
        self._stacked_widget.addWidget(self._registration_page)
        self._registration_page.cancelRegistration.connect(self.cancel_to_idle)
        self._registration_page.setRegistrationDetails.connect(self.intake_user_reg)

        self._confirmation_page = ConfirmationPage(self.enterConfirmation)
        self._stacked_widget.addWidget(self._confirmation_page)
        self._confirmation_page.cancelConfirmation.connect(self.cancel_to_idle)
        self._confirmation_page.retakeProfile.connect(self.retake_user_info)
        self._confirmation_page.confirmUserInfo.connect(self.proceed_to_order)

        self._order_page = OrderPage(self.enterOrder)
        self._stacked_widget.addWidget(self._order_page)
        self._order_page.cancelOrder.connect(self.cancel_to_idle)
        self._order_page.dispatchOrder.connect(self.dispatch_order)

    def go_to_page(self, index):
        self.current_page_index = index
        self._stacked_widget.setCurrentIndex(index)

    def leave_idle(self):
        #TODO: add logic here to select based on recoged/unrecoged face!
        #TODO: need better scheme for indexing pages in __init__
        self.go_to_page(1) #hardcoding reg page index
        self.enterRegistration.emit()
    
    def cancel_to_idle(self):
        self.go_to_page(0) #TODO: fix widget hardcode
        self.enterIdle.emit()

    @Slot(str,str,QImage)
    def intake_user_reg(self, first_name, last_name, profile_picture):
        self.go_to_page(2)
        self.enterConfirmation.emit(first_name, last_name, profile_picture)

    def retake_user_info(self):
        self.go_to_page(1)
        self.enterRegistration.emit()

    def proceed_to_order(self):
        self.go_to_page(3)
        self.enterOrder.emit()

    @Slot(int)
    def dispatch_order(self, order_id):
        print(f"ROS is a go for order_id={order_id}")
        self.ros_order_pub.publish(std_msgs.msg.UInt16(order_id))

    def frame_callback(self, image_msg):
        #only do this if we're on the reg page
        if self.current_page_index == 1:
            #get frame from msg
            frame = self.bridge.imgmsg_to_cv2(image_msg)

            #transform frame
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame = cv2.flip(frame, 1)

            #convert frame to img
            h, w, ch = frame.shape
            img = QImage(frame.data, w, h, ch*w, QImage.Format_RGB888)

            #size down img
            img = img.scaled(PICTURE_VIEW_WIDTH, PICTURE_VIEW_HEIGHT, Qt.KeepAspectRatio)

            #emit img
            self.updateFrame.emit(img)


if __name__ == "__main__":
    order_pub = rospy.Publisher("order", std_msgs.msg.UInt16)

    print("Blocking until registered with ROS master...")
    rospy.init_node("kiosk")
    print("Success! Registered with ROS master")

    app = QApplication()
    window = MainWindow(order_pub)
    rospy.Subscriber("frame", sensor_msgs.msg.Image, window.frame_callback)
    window.show() #TODO: Make fullscreen after debug!
    sys.exit(app.exec_())
