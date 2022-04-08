#!/usr/bin/env python3
import face_recognition
import cv2
import numpy as np
import os
import rospy
from time import time_ns

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

    def __init__(self):
        self.known_face_encodings = []
        self.known_face_names = []

        if USE_TEST_IMAGES:
            for face_image_filename in os.listdir(TEST_IMAGES_DIR):
                image = face_recognition.load_image_file(os.path.join(TEST_IMAGES_DIR, face_image_filename))
                
                face_encoding = face_recognition.face_encodings(image)[0]

                self.known_face_encodings.append(face_encoding)
                self.known_face_names.append(face_image_filename) #TODO: actually get name
        else:
            raise(Exception("UNIMPLEMENTED"))

        self.face_locations = []
        self.face_encodings = []
        self.face_names = []

    def loop(self, frame):
        '''Main recog loop.'''
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

        return True

    def show_names(self):
        print(self.face_names)


def main():
    time_last_processed_ns = 0
    process_period_ns = 1e9
    recog = ArnieRecognizer()

    def frame_callback(data):
        now_ns = time_ns()
        if now_ns > time_last_processed_ns + process_period_ns:
            return
        frame = data.data
        recog.loop(frame)
        time_last_processed = now_ns

    rospy.Subscriber("frame", frame_callback)
    rospy.init_node(NAME)
    rospy.spin()


if __name__ == "__main__":
    main()
