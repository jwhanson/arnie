'''Face Recognition Module for Arnie
Jonathan Hanson

This module implements all face_recognition processing for Arnie. The flow is:
this node subscribes to the "frame" topic, once per second runs the
do_recogntion() method to identify faces, and publishes the faces it sees to
the "recoged_names" topic.

An interesting design decision: this module really does subscribe to every
frame published, but has a timer that only runs the recognition sequence once
per second. This is cleaner than a) fiddling with subscribing and
unsubscribing, and b) somehow slowing down the subscriber.
'''
#!/usr/bin/env python3
import sys
import face_recognition
import cv2
import numpy as np
import os
import rospy
import std_msgs.msg
import sensor_msgs.msg
from cv_bridge import CvBridge
from time import time_ns
from arnie_kiosk.srv import (
    FetchUser,
    FetchIds
)
from arnie_vision.srv import (
    AddFaceToRecog, AddFaceToRecogResponse
)

NAME = "recognizer"
USE_TEST_IMAGES = True
TEST_IMAGES_DIR = os.path.join(
    os.getenv("ROS_PACKAGE_PATH").split(':')[0],
    'arnie',
    'arnie_vision',
    'faces'
)

class ArnieRecognizer(object):
    '''Arnie face recognizer object.'''

    def __init__(self, names_pub, fetch_ids_sh=None, fetch_user_sh=None):
        self.names_pub = names_pub
        self.fetch_ids_sh = fetch_ids_sh
        self.fetch_user_sh = fetch_user_sh
        self.time_last_processed_ns = 0
        self.process_period_ns = int(1e9)
        self.bridge = CvBridge()

        self.known_face_encodings = []
        self.known_face_names = []

        if fetch_user_sh == None or fetch_ids_sh == None:
            #use faces dir for debug
            for face_image_filename in os.listdir(TEST_IMAGES_DIR):
                image = face_recognition.load_image_file(os.path.join(TEST_IMAGES_DIR, face_image_filename))
                
                face_encoding = face_recognition.face_encodings(image)[0]

                self.known_face_encodings.append(face_encoding)
                self.known_face_names.append(face_image_filename) #TODO: actually get name
        else:
            #get going from the database
            try:
                fetch_ids_response = self.fetch_ids_sh()
                ids = fetch_ids_response.ids
                # #check response data 
                # print(f"({type(ids)}) ids: {ids}")
                for user_id in ids:
                    fetch_user_response = self.fetch_user_sh(user_id)
                    first_name = fetch_user_response.first_name.data
                    last_name = fetch_user_response.last_name.data
                    profile_picture_msg = fetch_user_response.profile_picture
                    profile_picture = self.bridge.imgmsg_to_cv2(profile_picture_msg)
                    profile_picture = cv2.cvtColor(profile_picture, cv2.COLOR_BGR2RGB)

                    face_encoding = face_recognition.face_encodings(profile_picture)[0]
                    self.known_face_encodings.append(face_encoding)
                    self.known_face_names.append(first_name+"_"+last_name)
            except rospy.ServiceException as e:
                print(f"Service call failed: {e}")

        self.face_locations = []
        self.face_encodings = []
        self.face_names = []

    def do_recognition(self, frame):
        '''Main recognition sequence.'''
        if not self.known_face_encodings:
            return False

        #resize to 1/4 for faster processing
        small_frame = cv2.resize(frame, (0,0), fx=0.25, fy=0.25)

        #convert from BGR to RGB
        rgb_small_frame = small_frame[:,:,::-1]

        #find all the faces and face encodings
        self.face_locations = face_recognition.face_locations(rgb_small_frame)
        self.face_encodings = face_recognition.face_encodings(rgb_small_frame, self.face_locations)

        self.face_names = []
        for face_encoding in self.face_encodings:
            #see if the face is a match for known faces
            matches = face_recognition.compare_faces(self.known_face_encodings, face_encoding)
            name = "Unknown"

            #use known face with the smallest dist to the new face
            face_distances = face_recognition.face_distance(self.known_face_encodings, face_encoding)
            best_match_index = np.argmin(face_distances)
            if matches[best_match_index]:
                name = self.known_face_names[best_match_index]
            
            self.face_names.append(name)

        self.show_names()
        return True

    def show_names(self):
        print(self.face_names)
        self.names_pub.publish(std_msgs.msg.String(str(self.face_names)))

    def frame_callback(self, image_msg):
        '''ROS Subscriber callback for a sensor_msgs.msg.Image message.

        Note: for frames published faster than process_period_ns, this 
        function simply returns! If frame publishing is slower, great, we
        handle every frame. If frame publishing is faster, we just dismiss
        the message to avoid taxing the Raspberry Pi with constant face
        recognizing.
        '''
        now_ns = time_ns()
        if now_ns < self.time_last_processed_ns + self.process_period_ns:
            return
        frame = self.bridge.imgmsg_to_cv2(image_msg)
        self.do_recognition(frame)
        self.time_last_processed_ns = now_ns
    
    def add_new_face(self, request):
        name = request.first_name+"_"+request.last_name
        profile_picture_msg = request.profile_picture
        profile_picture = self.bridge.imgmsg_to_cv2(profile_picture_msg)
        profile_picture = cv2.cvtColor(profile_picture, cv2.COLOR_BGR2RGB)
        face_encoding = face_recognition.face_encodings(profile_picture)[0]
        self.known_face_encodings.append(face_encoding)
        self.known_face_names.append(name)
        return AddFaceToRecogResponse(True)

def main():
    pub = rospy.Publisher("recoged_names", std_msgs.msg.String)

    print("Blocking until database services are available...")
    rospy.wait_for_service('fetch_ids')
    rospy.wait_for_service('fetch_user')
    print("Success! Database services are ready")

    #create service handle for calling service
    fetch_ids_sh = rospy.ServiceProxy('fetch_ids', FetchIds)
    fetch_user_sh = rospy.ServiceProxy('fetch_user', FetchUser)

    recog = ArnieRecognizer(pub, fetch_ids_sh, fetch_user_sh)

    rospy.init_node(NAME)
    rospy.Subscriber("frame", sensor_msgs.msg.Image, recog.frame_callback)
    rospy.Service('add_face_to_recog', AddFaceToRecog, recog.add_new_face)
    rospy.spin()


if __name__ == "__main__":
    main()
