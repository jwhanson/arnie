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
import numpy as np
import cv2
import rospy
import std_msgs.msg
import sensor_msgs.msg
from cv_bridge import CvBridge
from arnie_kiosk.srv import (
    InsertUser,
    InsertOrder,
    FetchItemIds,
    FetchItem
)
from arnie_vision.srv import (
    AddFaceToRecog
)


PICTURE_VIEW_WIDTH = 320
PICTURE_VIEW_HEIGHT = 240
TITLE_FONT_SIZE = 16
BODY_FONT_SIZE = 12
BUTTON_HEIGHT = 48

# https://stackoverflow.com/questions/18406149/pyqt-pyside-how-do-i-convert-qimage-into-opencvs-mat-format
def convertQImageToMat(incomingImage):
    '''Converts a QImage into an opencv MAT format'''

    incomingImage = incomingImage.convertToFormat(QImage.Format_RGB32)
    width = incomingImage.width()
    height = incomingImage.height()
    data = incomingImage.constBits()
    arr = np.asarray(data)
    image_arr = np.copy(arr).reshape(height,width,4)

    return image_arr


class RosThread(QThread):
    def __init__(self):
        super().__init__()
    
    def run(self):
        rospy.spin()


class StartPage(QWidget):
    """PySide Widget implementing the start page for Arnie."""
    goToRegistration = Signal()
    loginOrder = Signal()
    guestOrder = Signal()

    def __init__(self, enterPageSignal, updateRecognitionSignal):
        super().__init__()

        enterPageSignal.connect(self.enter)
        updateRecognitionSignal.connect(self.update_recognition)

        # Title Text | QLabel
        self.title_text = QLabel("ArnieBot - The Original Iced Tea + Lemonade\nBeverage Service System")
        font = self.title_text.font()
        font.setPointSize(TITLE_FONT_SIZE)
        self.title_text.setFont(font)
        self.title_text.setAlignment(Qt.AlignHCenter|Qt.AlignVCenter)

        # Subtitle Text | QLabel
        self.recognition_text = QLabel("No faces detected...")
        font = self.recognition_text.font()
        font.setPointSize(BODY_FONT_SIZE)
        self.recognition_text.setFont(font)
        self.recognition_text.setAlignment(Qt.AlignHCenter|Qt.AlignVCenter)

        # Register Button | QPushButton
        self.register_button = QPushButton("Register")
        self.register_button.setFixedHeight(BUTTON_HEIGHT)
        self.register_button.clicked.connect(self.register)

        # Login Button | QPushButton
        self.login_button = QPushButton("Login")
        self.login_button.setFixedHeight(BUTTON_HEIGHT)
        self.login_button.setEnabled(False) #disabled until user is recognized
        self.login_button.clicked.connect(self.login)

        # Guest Button | QPushButton
        self.guest_button = QPushButton("Guest")
        self.guest_button.setFixedHeight(BUTTON_HEIGHT)
        self.guest_button.clicked.connect(self.guest_order)

        # Layout for bottom row of buttons
        button_layout = QHBoxLayout()
        button_layout.addWidget(self.register_button)
        button_layout.addWidget(self.login_button)
        button_layout.addWidget(self.guest_button)

        # Overall layout
        main_layout = QVBoxLayout()
        main_layout.addWidget(self.title_text)
        main_layout.addWidget(self.recognition_text)
        main_layout.addLayout(button_layout)
        self.setLayout(main_layout)
    
    def enter(self):
        print('entering idle page')

    def leave(self):
        print('leaving idle page')

    @Slot(str)
    def update_recognition(self, name_str):
        if name_str == '_empty':
            self.login_button.setEnabled(False)
            self.recognition_text.setText("No faces detected...")
        elif name_str == '_multiple':
            self.login_button.setEnabled(False)
            self.recognition_text.setText("Too many faces in frame!")
        elif name_str == '_unknown':
            self.login_button.setEnabled(False)
            self.recognition_text.setText("Welcome new user!")
        else:
            self.login_button.setEnabled(True)
            self.recognition_text.setText(f"Welcome back, {name_str}")

        self.repaint()

    def register(self):
        self.leave()
        self.goToRegistration.emit()
    
    def login(self):
        self.leave()
        self.loginOrder.emit()

    def guest_order(self):
        self.leave()
        self.guestOrder.emit()


class RegistrationPage(QWidget):
    """PySide2 Widget implementing the user registration page for Arnie."""
    
    cancelRegistration = Signal()
    setRegistrationDetails = Signal(str,str,QImage)

    def __init__(self, enterPageSignal, updateFrameSignal):
        super().__init__()

        enterPageSignal.connect(self.enter)
        updateFrameSignal.connect(self.set_image)

        # Greeting Text | QLabel
        self.greeting_text = QLabel("Welcome new user!\nPlease enter your name\nand take a profile picture")
        font = self.greeting_text.font()
        font.setPointSize(BODY_FONT_SIZE)
        self.greeting_text.setFont(font)
        self.greeting_text.setAlignment(Qt.AlignHCenter|Qt.AlignVCenter)

        # First Name Input Label | QLabel
        self.first_name_label = QLabel("First Name:")
        self.first_name_label.setAlignment(Qt.AlignLeft|Qt.AlignBottom)
        self.first_name_label.setFixedHeight(24)

        # First Name Input Textbox | QLineEdit
        self.first_name_edit = QLineEdit("Arnold")
        self.first_name_edit.setFixedWidth(180)

        # Last Name Input Label | QLabel
        self.last_name_label = QLabel("Last Name:")
        self.last_name_label.setAlignment(Qt.AlignLeft|Qt.AlignBottom)
        self.last_name_label.setFixedHeight(24)

        # Last Name Input Textbox | QLineEdit
        self.last_name_edit = QLineEdit("Palmer")
        self.last_name_edit.setFixedWidth(180)

        # Spacer (blank) | QLabel
        self.spacer_label = QLabel("")

        # Take Picture Button | QPushButton
        self.take_picture_button = QPushButton("Take Picture")
        self.take_picture_button.setFixedHeight(BUTTON_HEIGHT)
        self.take_picture_button.clicked.connect(self.take_picture)
    
        # Cancel Button | QPushButton
        self.cancel_button = QPushButton("Cancel")
        self.cancel_button.setFixedHeight(BUTTON_HEIGHT)
        self.cancel_button.clicked.connect(self.cancel)

        # Layout for name input and buttons
        self.left_layout = QVBoxLayout()
        self.left_layout.addWidget(self.greeting_text)
        self.left_layout.addWidget(self.first_name_label)
        self.left_layout.addWidget(self.first_name_edit)
        self.left_layout.addWidget(self.last_name_label)
        self.left_layout.addWidget(self.last_name_edit)
        self.left_layout.addWidget(self.spacer_label)
        self.left_layout.addWidget(self.take_picture_button)
        self.left_layout.addWidget(self.cancel_button)

        # Camera Preview Widget | QLabel
        self.camera_view_label = QLabel(self)
        self.camera_view_label.setFixedSize(PICTURE_VIEW_WIDTH, PICTURE_VIEW_HEIGHT)

        # Camera Image | QImage
        self.camera_view_img = QImage()

        # Layout for full page
        main_layout = QHBoxLayout()
        main_layout.addLayout(self.left_layout)
        main_layout.addWidget(self.camera_view_label)
        self.setLayout(main_layout)

    def enter(self):
        print("entering reg page")
        self.camera_view_label.clear() #cleanup camera view widget

    def leave(self):
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
        
        self.leave()
        self.setRegistrationDetails.emit(first_name, last_name, self.camera_view_img)

    def cancel(self):
        self.leave()
        self.cancelRegistration.emit()


class ConfirmationPage(QWidget):
    """PySide2 Widget implementing a user profile review page for Arnie."""
    cancelConfirmation = Signal()
    retakeProfile = Signal()
    confirmUserInfo = Signal(str,str,QImage)

    def __init__(self, enterPageSignal):
        super().__init__()

        enterPageSignal.connect(self.enter)

        # Confirmation Text | Qlabel
        self._confirm_text = QLabel("Does this look OK?")
        font = self._confirm_text.font()
        font.setPointSize(BODY_FONT_SIZE)
        self._confirm_text.setFont(font)
        self._confirm_text.setAlignment(Qt.AlignHCenter|Qt.AlignVCenter)

        # Approve Button | QPushButton
        self._approve_button = QPushButton("Approve")
        self._approve_button.setFixedHeight(BUTTON_HEIGHT)
        self._approve_button.clicked.connect(self.approve)

        # Retake Button | QPushButton
        self._retake_button = QPushButton("Retake")
        self._retake_button.setFixedHeight(BUTTON_HEIGHT)
        self._retake_button.clicked.connect(self.retake)

        # Cancel Button | QPushButton
        self._cancel_button = QPushButton("Cancel")
        self._cancel_button.setFixedHeight(BUTTON_HEIGHT)
        self._cancel_button.clicked.connect(self.cancel)

        # Layout for buttons stacked vertically
        left_layout = QVBoxLayout()
        left_layout.addWidget(self._confirm_text)
        left_layout.addWidget(self._approve_button)
        left_layout.addWidget(self._retake_button)
        left_layout.addWidget(self._cancel_button)

        # Profile Picture Preivew Space | QLabel
        self._profile_picture = QLabel(self)
        self._profile_picture.setFixedSize(PICTURE_VIEW_WIDTH, PICTURE_VIEW_HEIGHT)
        
        # Profile Name Preview Text | Qlabel
        self._first_name = "NOINIT_FIRST" #if you see these, then enter() failed
        self._last_name = "NOINIT_LAST"
        self._profile_name = QLabel(f"Name: '{self._first_name} {self._last_name}'")
        font = self._profile_name.font()
        font.setPointSize(BODY_FONT_SIZE)
        self._profile_name.setFont(font)
        self._profile_name.setAlignment(Qt.AlignHCenter | Qt.AlignTop)
        self._profile_name.setFixedHeight(32) #TODO: fix size hardcode

        # Layout for pic and name preview
        right_layout = QVBoxLayout()
        right_layout.addWidget(self._profile_picture)
        right_layout.addWidget(self._profile_name)

        # Layout for full page
        main_layout = QHBoxLayout()
        main_layout.addLayout(left_layout)
        main_layout.addLayout(right_layout)
        self.setLayout(main_layout)

    @Slot(str,str,QImage)
    def enter(self, first_name, last_name, profile_picture):
        print("entering conf page")
        self._profile_picture.clear()

        self._first_name = first_name
        self._last_name = last_name
        self._profile_picture_qimage = profile_picture
        self._profile_name.setText(f"Name: '{self._first_name} {self._last_name}'")

        self._profile_picture.setPixmap(QPixmap.fromImage(profile_picture))

    def leave(self):
        print("leaving conf page")
    
    def approve(self):
        print("Approved!")
        self.leave()
        self.confirmUserInfo.emit(self._first_name, self._last_name, self._profile_picture_qimage)

    def retake(self):
        self.leave()
        self.retakeProfile.emit()

    def cancel(self):
        self.leave()
        self.cancelConfirmation.emit()


class MenuButton(QPushButton):
    def __init__(self, button_text):
        super().__init__(button_text)
        self.setFixedHeight(90) #TODO no hard code?
        self.setEnabled(False)


class SpecialMenu(QWidget):
    def __init__(self):
        super().__init__()

        #TODO: something special
        # for now just put menu item 1

        # Special Item Button | MenuButton
        self.special_button = MenuButton('S')

        # Special Item Text | QLabel
        self.special_text = QLabel("could be specialer")

        special_layout = QVBoxLayout()
        special_layout.addWidget(self.special_button)
        special_layout.addWidget(self.special_text)

        self.setLayout(special_layout)


class NormalMenu(QWidget):
    def __init__(self, fetch_menu_item_ids_sh, fetch_menu_item_sh):
        super().__init__()
        
        # Setup menu via ROS service handle database calls
        self.menu = dict()
        response = fetch_menu_item_ids_sh()
        item_ids = response.item_ids
        for item_id in item_ids:
            response = fetch_menu_item_sh(item_id)
            self.menu[item_id] = response.name

        # Configure Button Layout dimentions
        num_rows = 2
        num_cols = 3

        # Setup Generic MenuButtons
        self.menu_buttons = []
        for i in range(num_rows*num_cols):
            menu_button = MenuButton(f"{i+1}")
            self.menu_buttons.append(menu_button)


        # Upper layout (3 buttons in a row)
        upper_layout = QHBoxLayout()
        for i in range(num_cols):
            upper_layout.addWidget(self.menu_buttons[i])

        # Lower layout (3 buttons in a row)
        lower_layout = QHBoxLayout()
        for i in range(num_cols):
            lower_layout.addWidget(self.menu_buttons[i+num_cols])

        # Main layout (2 rows of buttons)
        menu_layout = QVBoxLayout()
        menu_layout.addLayout(upper_layout)
        menu_layout.addLayout(lower_layout)

        # Scroll Menu Left Button | QPushButton
        self._scroll_left_button = QPushButton("<")
        self._scroll_left_button.setFixedHeight(200)
        self._scroll_left_button.setFixedWidth(32)
        self._scroll_left_button.clicked.connect(self.scroll_left)

        # Scroll Menu Right Button | QPushButton
        self._scroll_right_button = QPushButton(">")
        self._scroll_right_button.setFixedHeight(200)
        self._scroll_right_button.setFixedWidth(32)
        self._scroll_right_button.clicked.connect(self.scroll_right)

        # Main Layout
        main_layout = QHBoxLayout()
        main_layout.addWidget(self._scroll_left_button)
        main_layout.addLayout(menu_layout)
        main_layout.addWidget(self._scroll_right_button)
        self.setLayout(main_layout)

        # Setup manipulation of menu
        self.current_page = 0
        self.items_per_page = num_rows*num_cols
        self.num_pages = len(self.menu)//self.items_per_page

        # Populate Menu
        self.redraw_menu()

    def set_page(self, page_num):
        if page_num >=0 and page_num < self.num_pages:
            self.current_page = page_num
            self.redraw_menu()

    def redraw_menu(self):
        if self.current_page == 0:
            self._scroll_left_button.setEnabled(False)
        else:
            self._scroll_left_button.setEnabled(True)
        if self.current_page == self.num_pages:
            self._scroll_right_button.setEnabled(False)
        else:
            self._scroll_right_button.setEnabled(True)

        for i in range(self.items_per_page):
            menu_index = i + self.current_page*self.items_per_page
            if menu_index < len(self.menu):
                item_id = list(self.menu.keys())[menu_index]
                self.menu_buttons[i].setText(self.menu[item_id])
                self.menu_buttons[i].setEnabled(True)
            else:
                self.menu_buttons[i].setText("N/A")
                self.menu_buttons[i].setEnabled(False)

    def scroll_left(self):
        self.current_page -= 1
        self.redraw_menu()

    def scroll_right(self):
        self.current_page += 1
        self.redraw_menu()


class OrderPage(QWidget):
    '''PySide widget implementing a simple demo ordering system.'''
    cancelOrder = Signal()
    dispatchOrder = Signal(int)

    def __init__(self, enterPageSignal, fetch_menu_item_ids_sh, fetch_menu_item_sh):
        super().__init__()

        enterPageSignal.connect(self.enter)

        # Top Text | QLabel
        self._order_text = QLabel("Thanks #NAME#, please select a beverage and tap 'Submit'") #TODO: add name
        font = self._order_text.font()
        font.setPointSize(BODY_FONT_SIZE)
        self._order_text.setFont(font)
        self._order_text.setAlignment(Qt.AlignVCenter) #default align left?

        # Special Menu Object | QWidget (Custom)
        self.special_menu = SpecialMenu()

        # Normal Menu Object | QWidget (Custom)
        self.normal_menu = NormalMenu(fetch_menu_item_ids_sh, fetch_menu_item_sh)

        # Core Menu Layout (special menu left, normal menu right)
        core_layout = QHBoxLayout()
        core_layout.addWidget(self.special_menu)
        core_layout.addWidget(self.normal_menu)

        # Submit Button | QPushButton
        self._spacer = QLabel("")

        # Cancel Button | QPushButton
        self._cancel_button = QPushButton("Cancel")
        self._cancel_button.setFixedHeight(BUTTON_HEIGHT)
        self._cancel_button.clicked.connect(self.cancel)

        # Layout for cancel button
        bottom_button_layout = QHBoxLayout()
        bottom_button_layout.addWidget(self._spacer)
        bottom_button_layout.addWidget(self._cancel_button)

        # Main Layout
        main_layout = QVBoxLayout()
        main_layout.addWidget(self._order_text)
        main_layout.addLayout(core_layout)
        main_layout.addLayout(bottom_button_layout)
        self.setLayout(main_layout)

    @Slot(int)
    def enter(self, user_id):
        print(f"entering order page; curr user {user_id}")
        self.normal_menu.set_page(0) #reset the normal menu widget

    def leave(self):
        print("leaving order page")
    
    def submit(self):
        #TODO
        order_id = "TODO"
        self.dispatchOrder.emit(order_id)

    def cancel(self):
        self.leave()
        self.cancelOrder.emit()


class WaitPage(QWidget):
    '''PySide widget implementing a simple wait screen.'''

    returnToMenu = Signal()

    def __init__(self, enterPageSignal, doneWaitingSignal):
        super().__init__()

        enterPageSignal.connect(self.enter)
        doneWaitingSignal.connect(self.done_waiting)
        
        #controls whether user can tap to continue
        self.served = False

        # Header Wait Text | QLabel
        self._wait_header_text = QLabel("Please wait,")
        font = self._wait_header_text.font()
        font.setPointSize(TITLE_FONT_SIZE)
        self._wait_header_text.setFont(font)
        self._wait_header_text.setAlignment(Qt.AlignHCenter|Qt.AlignVCenter)

        # Body Wait Text | QLabel
        self._wait_body_text = QLabel("Arnie is preparing your beverage")
        font = self._wait_body_text.font()
        font.setPointSize(BODY_FONT_SIZE)
        self._wait_body_text.setFont(font)
        self._wait_body_text.setAlignment(Qt.AlignHCenter|Qt.AlignVCenter)

        # Set Layout
        layout = QVBoxLayout()
        layout.addWidget(self._wait_header_text)
        layout.addWidget(self._wait_body_text)
        self.setLayout(layout)

    def enter(self):
        print("entering wait page")
        self.served = False

    def leave(self):
        print("leaving wait page")

    def done_waiting(self):
        self.served = True

        # update text
        self._wait_header_text.setText("Enjoy!")
        font = self._wait_header_text.font()
        font.setFamily("Pacifico")
        font.setPointSize(72) #TODO: fix hardcoded size
        self._wait_header_text.setFont(font)

        self._wait_body_text.setText("Tap anywhere to return to the main menu.")
        font = self._wait_body_text.font()
        font.setPointSize(BODY_FONT_SIZE)
        self._wait_body_text.setFont(font)
    
    def mousePressEvent(self, event):
        if self.served == True:
            print("click...")
            self.leave()
            self.returnToMenu.emit()


class MainWindow(QMainWindow):
    current_page_index = 0

    updateFrame = Signal(QImage)
    updateRecognition = Signal(str)

    doneWaiting = Signal()

    enterStart = Signal()
    enterRegistration = Signal()
    enterConfirmation = Signal(str,str,QImage)
    enterOrder = Signal(int)
    enterWait = Signal()

    def __init__(self, ros_order_pub, insert_user_sh, insert_order_sh, add_face_to_recog_sh, fetch_menu_item_ids_sh, fetch_menu_item_sh):
        super().__init__()
        # ROS setup
        self.bridge = CvBridge()
        self.ros_order_pub = ros_order_pub
        self.insert_user_sh = insert_user_sh
        self.insert_order_sh = insert_order_sh
        self.add_face_to_recog_sh = add_face_to_recog_sh
        self.ros_thread = RosThread()
        self.ros_thread.start()

        # Tracks user_id of active user
        self.active_user_id = None

        self.setWindowTitle("Arnie Kiosk")

        # Stacked Widget is how we organize pages
        self._stacked_widget = QStackedWidget()
        self.setCentralWidget(self._stacked_widget)

        # Page 0 | Start Page
        self._start_page = StartPage(self.enterStart, self.updateRecognition)
        self._stacked_widget.addWidget(self._start_page)
        self._start_page.goToRegistration.connect(self.go_to_registration)
        self._start_page.loginOrder.connect(self.login_and_order)
        self._start_page.guestOrder.connect(self.guest_order)

        # Page 1 | Registration Page
        self._registration_page = RegistrationPage(self.enterRegistration, self.updateFrame)
        self._stacked_widget.addWidget(self._registration_page)
        self._registration_page.cancelRegistration.connect(self.go_to_start)
        self._registration_page.setRegistrationDetails.connect(self.intake_user_reg)

        # Page 2 | Confirmation Page
        self._confirmation_page = ConfirmationPage(self.enterConfirmation)
        self._stacked_widget.addWidget(self._confirmation_page)
        self._confirmation_page.cancelConfirmation.connect(self.go_to_start)
        self._confirmation_page.retakeProfile.connect(self.retake_user_info)
        self._confirmation_page.confirmUserInfo.connect(self.insert_user_info)

        # Page 3 | Order Page
        self._order_page = OrderPage(self.enterOrder, fetch_menu_item_ids_sh, fetch_menu_item_sh)
        self._stacked_widget.addWidget(self._order_page)
        self._order_page.cancelOrder.connect(self.go_to_start)
        self._order_page.dispatchOrder.connect(self.dispatch_order)

        # Page 4 | Wait Page
        self._wait_page = WaitPage(self.enterWait, self.doneWaiting)
        self._stacked_widget.addWidget(self._wait_page)
        self._wait_page.returnToMenu.connect(self.go_to_start)

    def go_to_page(self, index):
        """Simple helper function to track the page index."""
        self.current_page_index = index
        self._stacked_widget.setCurrentIndex(index)

    def go_to_start(self):
        self.go_to_page(0)
        self.active_user_id = 0 #reset the active user at the start page
        self.enterStart.emit()

    def go_to_registration(self):
        self.go_to_page(1) #hardcoding reg page index
        self.enterRegistration.emit()
    
    def login_and_order(self):
        if self.active_user_id != None:
            self.go_to_page(3) #hardcoding order page index
            self.enterOrder.emit(self.active_user_id)

    def guest_order(self):
        self.go_to_page(3) #hardcoding order page index
        self.enterOrder.emit(0) #zero represents the guest user

    @Slot(str,str,QImage)
    def intake_user_reg(self, first_name, last_name, profile_picture):
        self.go_to_page(2)
        self.enterConfirmation.emit(first_name, last_name, profile_picture)

    def retake_user_info(self):
        self.go_to_page(1)
        self.enterRegistration.emit()

    @Slot(str,str,QImage)
    def insert_user_info(self, first_name, last_name, profile_picture_qimage):
        profile_picture_cv = convertQImageToMat(profile_picture_qimage)
        print(type(profile_picture_cv))
        profile_picture_msg = self.bridge.cv2_to_imgmsg(cvim=profile_picture_cv, encoding="passthrough")
        try:
            response = self.insert_user_sh(first_name, last_name, profile_picture_msg)
            user_id = response.user_id
            print(f"({type(user_id)})user_id: {user_id}")
            self.add_face_to_recog_sh(user_id, first_name, last_name, profile_picture_msg)
            self.go_to_page(3)
            self.enterOrder.emit(user_id)
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
            self.go_to_start()

    @Slot(int)
    def dispatch_order(self, item_id):
        # If this is a registered user, then try the database interaction
        if self.active_user_id > 0:
            try:
                response = self.insert_order_sh(self.active_user_id, item_id)
                order_id = response.order_id
                print(f"({type(order_id)})order_id: {order_id}")
            except rospy.ServiceException as e:
                print(f"Service call failed: {e}")
                self.go_to_start()
                return

        #publish the new order to ROS
        self.ros_order_pub.publish(std_msgs.msg.UInt16(order_id))
        print(f"ROS is a go for order_id={order_id}")
        self.go_to_page(4) #wait page
        self.enterWait.emit()

    def frame_callback(self, image_msg):
        '''ROS Sub Callback for the /frame topic'''
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
    
    def recoged_names_callback(self, string_msg):
        '''ROS Sub Callback for the /recoged_names topic'''
        #only do this if we're on the start page
        if self.current_page_index == 0:
            recoged_names_messy = string_msg.data.strip('][').split(', ') #split out names
            recoged_names = [name.strip('\'"') for name in recoged_names_messy] #strip off quotes
            if '' in recoged_names:
                recoged_names.remove('')

            if len(recoged_names) == 0:
                self.updateRecognition.emit('_empty')
            elif len(recoged_names) == 1:
                long_name = recoged_names_messy[0].strip('\'"') #strip off quotes
                if long_name == 'Unknown':
                    self.updateRecognition.emit("_unknown")
                else:
                    user_id, first_name, last_name = long_name.split('_')
                    self.active_user_id = int(user_id)
                    self.updateRecognition.emit(" ".join([first_name, last_name]))
            elif len(recoged_names) == 2:
                self.updateRecognition.emit('_multiple')
    
    def served_callback(self, served_msg):
        '''ROS Sub Callback for the /served topic'''
        #only do this if we're on the wait page
        if self.current_page_index == 4:
            if served_msg.data == True:
                self.doneWaiting.emit()


if __name__ == "__main__":
    order_pub = rospy.Publisher("order", std_msgs.msg.UInt16)

    print("Blocking until registered with ROS master...")
    rospy.init_node("kiosk")
    print("Success! Registered with ROS master")

    print("Blocking until database services are available...")
    rospy.wait_for_service('insert_user')
    rospy.wait_for_service('add_face_to_recog')
    rospy.wait_for_service('fetch_item_ids')
    rospy.wait_for_service('fetch_item')
    print("Success! Database services are ready")

    #create service handle for calling service
    insert_user_sh = rospy.ServiceProxy('insert_user', InsertUser)
    insert_order_sh = rospy.ServiceProxy('insert_order', InsertOrder)
    add_face_to_recog_sh = rospy.ServiceProxy('add_face_to_recog', AddFaceToRecog)
    fetch_item_ids_sh = rospy.ServiceProxy('fetch_item_ids', FetchItemIds)
    fetch_item_sh = rospy.ServiceProxy('fetch_item', FetchItem)

    app = QApplication()
    window = MainWindow(order_pub, insert_user_sh, insert_order_sh, add_face_to_recog_sh, fetch_item_ids_sh, fetch_item_sh)
    rospy.Subscriber("frame", sensor_msgs.msg.Image, window.frame_callback)
    rospy.Subscriber("recoged_names", std_msgs.msg.String, window.recoged_names_callback)
    rospy.Subscriber("served", std_msgs.msg.Bool, window.served_callback)
    window.show() #TODO: Make fullscreen after debug!
    sys.exit(app.exec_())
