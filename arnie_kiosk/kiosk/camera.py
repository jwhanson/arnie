import cv2
import os

TEMP_IMAGE_FILENAME = "tmp.jpg"

class ArnieCameraManager(object):
    def __init__(self, camera_device_id):
        self.device_id = camera_device_id

    def start_capture(self):
        """Helper function to start a new VideoCapture as necessary."""
        self._cap = cv2.VideoCapture(self.device_id)

    def take_user_picture(self):
        """Take a profile picture with OpenCV and produce a binary blob for storage.

        Params:
            device_id: OpenCV VideoCapture device ID.
        
        Returns:
            (bytes) A binary blob of the profile pic taken.
        """
        if not self._cap.isOpened():
            self.start_capture()

        while self._cap.isOpened():
            ret, frame = self._cap.read()
            if not ret:
                continue

            cv2.imshow("User Picture Preview", frame)

            if cv2.waitKey(5) & 0xFF == ord('q'):
                break

        self._cap.release()
        cv2.destroyAllWindows()

        cv2.imwrite(TEMP_IMAGE_FILENAME, frame)

        img_blob = None
        with open(TEMP_IMAGE_FILENAME, "rb") as infile:
            img_blob = infile.read()

        os.remove(TEMP_IMAGE_FILENAME)
        return img_blob


    def display_image_blob(self, name, img_blob):
        with open(TEMP_IMAGE_FILENAME, "wb") as outfile:
            outfile.write(img_blob)

        pic = cv2.imread(TEMP_IMAGE_FILENAME)

        cv2.imshow(f"Profile Pic for '{name}'", pic)

        while True:
            if cv2.waitKey(5) & 0xFF == ord('q'):
                break

        os.remove(TEMP_IMAGE_FILENAME)

        return
