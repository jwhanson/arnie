#!/usr/bin/env python3
import face_recognition
import cv2
import numpy as np
import os

CAMERA_DEVICE_ID = 0
USE_TEST_IMAGES = True
TEST_IMAGES_DIR = os.path.join(
    os.getenv("ROS_PACKAGE_PATH").split(':')[0],
    'arnie',
    'arnie_recognizer',
    'faces'
)

class ArnieRecognizer(object):
    '''Arnie face recognizer object.'''

    def __init__(self):
        self.cap = cv2.VideoCapture(CAMERA_DEVICE_ID)
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
        self.process_this_frame = True
    
    def __del__(self):
        print("running destuctor")
        self.cap.release()
        cv2.destroyAllWindows()

    def loop(self):
        '''Main recog loop.'''
        #grab a frame
        success, frame = self.cap.read()
        if not success:
            return True

        #resize to 1/4 for faster processing
        small_frame = cv2.resize(frame, (0,0), fx=0.25, fy=0.25)

        #convert from BGR to RGB
        rgb_small_frame = small_frame[:,:,::-1]

        #process every other frame
        if self.process_this_frame:
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

        self.process_this_frame = not self.process_this_frame

        return True

    def show_names(self):
        print(self.face_names)


if __name__ == "__main__":
    rec = ArnieRecognizer()
    success = True

    while success:
        success = rec.loop()
        rec.show_names()
