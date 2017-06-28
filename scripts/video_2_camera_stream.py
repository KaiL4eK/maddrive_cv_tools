#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError

rospy.init_node('video_2_camera_stream')

videoPub = rospy.Publisher('/camera/rgb/image_color', Image, queue_size=10)
rate = rospy.Rate(30)

filepath = rospy.get_param('~filepath')

rospy.loginfo('Filepath: %s' % filepath)

capture = cv2.VideoCapture(filepath)

if not capture.isOpened():
    rospy.logerr('Failed to open file')
    exit(1)

capture.set(3,640);
capture.set(4,480);

bridge = CvBridge()

def main():
    while not rospy.is_shutdown():
        meta, frame = capture.read()

        if frame is None:
            rospy.loginfo('frame is None')
            capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
            continue

        # frame_gaus = cv2.GaussianBlur(frame, (3, 3), 0)
        # frame_gray = cv2.cvtColor(frame_gaus, cv2.COLOR_BGR2GRAY)

        # I want to publish the Canny Edge Image and the original Image
        videoPub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo('Exception caught')