#!/usr/bin/python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from image_distort import ImageLenseDistort
import numpy as np


class SteroCameraNode:

    def __init__(self):
        self.stereo_publisher = None
        self.cv_bridge = None
        self.r_img = None
        self.l_img = None
        self.lense_distort = ImageLenseDistort()

        self.init_bindings()
        pass

    def r_img_callback(self, img):
        self.r_img = img
        pass

    def l_img_callback(self, img):
        self.l_img = img
        pass

    def init_bindings(self):
        rospy.init_node("stereo_cameras")

        # Subscribe for two "eyes"
        rospy.Subscriber("/openhmd/right/image_raw", Image, self.r_img_callback)
        rospy.Subscriber("/openhmd/right/image_raw", Image, self.l_img_callback)

        self.stereo_publisher = rospy.Publisher("/openhmd/stereo", Image, queue_size=1000)
        self.cv_bridge = CvBridge()
        pass

    def process(self):
        while not rospy.is_shutdown():
            if self.l_img is not None and self.r_img is not None:
                cv_right_image = self.cv_bridge.imgmsg_to_cv2(self.r_img, desired_encoding="bgr8")
                cv_left_image = self.cv_bridge.imgmsg_to_cv2(self.l_img, desired_encoding="bgr8")

                cv_right_image = self.lense_distort.process_frame(cv_right_image)
                cv_left_image = self.lense_distort.process_frame(cv_left_image)

                cv_stereo_image = np.append(cv_left_image, cv_right_image, axis=1)
                stereo_image = self.cv_bridge.cv2_to_imgmsg(cv_stereo_image, encoding="bgr8")

                self.stereo_publisher.publish(stereo_image)
        pass

if __name__ == "__main__":
    scam = SteroCameraNode()
    scam.process()