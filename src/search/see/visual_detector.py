#!/usr/bin/env

from __future__ import print_function
import sys
import cv2
import rospy
import os
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import mediapipe as mp
from photographer import Photographer
from threading import Lock

class Person_Detector_Photoshooter:

    face_cascade = cv2.CascadeClassifier(
        '/home/s/.local/lib/python3.8/site-packages/cv2/data/haarcascade_frontalface_default.xml'
    )
    body_cascade = cv2.CascadeClassifier(
        '/home/s/.local/lib/python3.8/site-packages/cv2/data/haarcascade_fullbody.xml'
    )

    # POSE ESTIMATOR
    mp_drawing = mp.solutions.drawing_utils
    mp_pose = mp.solutions.pose

    pose = mp_pose.Pose(
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5)

    # Variablen
    takePhoto = False
    img_counter = 0

    def __init__(self):
        self.pub = rospy.Publisher("output", Image, queue_size=10)

        self.bridge = CvBridge()

        self.is_taking_picture = False
        self.cam = Photographer()
        self.lock = Lock()

        # Turtlebot Raspi Camera subscriben
        self.sub = rospy.Subscriber("/raspicam_node/image/compressed",
                                    CompressedImage, self.callback)

    def callback(self, frame):
        with self.lock:
            is_taking = self.is_taking_picture

        if is_taking:
            return

        try:
            cv_Frame = self.bridge.compressed_imgmsg_to_cv2(frame, "bgr8")
        except CvBridgeError as e:
            print(e)

        # ---COMPUTATION---
        rgb = cv2.cvtColor(cv_Frame, cv2.COLOR_BGR2RGB)
        keypoints = self.pose.process(rgb)

        # DIE EINZELNEN LANDMARKS
        # NOSE = 0
        # LEFT_EYE_INNER = 1
        # LEFT_EYE = 2
        # LEFT_EYE_OUTER = 3
        # RIGHT_EYE_INNER = 4
        # RIGHT_EYE = 5
        # RIGHT_EYE_OUTER = 6
        # LEFT_EAR = 7
        # RIGHT_EAR = 8
        # MOUTH_LEFT = 9
        # MOUTH_RIGHT = 10
        # LEFT_SHOULDER = 11
        # RIGHT_SHOULDER = 12
        # LEFT_ELBOW = 13
        # RIGHT_ELBOW = 14
        # LEFT_WRIST = 15
        # RIGHT_WRIST = 16
        # LEFT_PINKY = 17
        # RIGHT_PINKY = 18
        # LEFT_INDEX = 19
        # RIGHT_INDEX = 20
        # LEFT_THUMB = 21
        # RIGHT_THUMB = 22
        # LEFT_HIP = 23
        # RIGHT_HIP = 24
        # LEFT_KNEE = 25
        # RIGHT_KNEE = 26
        # LEFT_ANKLE = 27
        # RIGHT_ANKLE = 28
        # LEFT_HEEL = 29
        # RIGHT_HEEL = 30
        # LEFT_FOOT_INDEX = 31
        # RIGHT_FOOT_INDEX = 32

        # NUR FUER DRAWING, NICHT VERARBEITEN
        lm = keypoints.pose_landmarks
        
        # DIE JEWEILIGEN LANDMARKS DES FRAMES
        try:
        # if True:
            print(".")
            landmarks = lm.landmark
            print("got through", flush=True)
            # value gibt dir den landmark selber
            noseV = landmarks[self.mp_pose.PoseLandmark.NOSE.value].visibility
            
            # Prüfen ob (Nase) visible
            # TODO Tweaken zu genug landmarks bzw. bestimmte Landmarks
            print("reached if")
            if landmarkVisible(noseV):
                # FOTO MACHEN = TRUE
                # self.takePhoto = True
                with self.lock:
                    self.is_taking_picture = True
                self.cam.record()
                print("Photo taken")
                rospy.sleep(20)
                with self.lock:
                    self.is_taking_picture = False
            else:
                print("no person")
            
            #print(landmarks)
            
        except:
            print("nothing to be seen")
        
        # LANDMARKS AUF FRAME ZEICHNEN
        self.mp_drawing.draw_landmarks(
            cv_Frame, lm, self.mp_pose.POSE_CONNECTIONS)

        # if self.img_counter < 3 and self.takePhoto:
        #     name = "photo_{}.png".format(self.img_counter)
        #     cv2.imwrite(os.path.join(self.path, name), cv_Frame)
        #     self.img_counter += 1

         #FENSTER ZEIGEN
        cv2.imshow("Turtech Video Feed", cv_Frame)
        cv2.waitKey(30) & ord('q')

        try:
            self.pub.publish(self.bridge.cv2_to_imgmsg(cv_Frame, "bgr8"))
        except CvBridgeError as e:
            print(e)


def landmarkVisible(value):
    if value > 0.9:
        return True

if __name__ == '__main__':
    Person_Detector_Photoshooter()

    # rospy.init_node('Person_Detector_Photoshooter', anonymous=True)

    try:
        print("spinning time")
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()
