#!/usr/bin/env python
 # let know what language is used

import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import matplotlib.pyplot as plt

rospy.init_node('traffic_light_region')
bridge = CvBridge()

window_name = 'image_for_searching_contours'
color_trackbar_Hmin = 'H__min'
color_trackbar_Hmax = 'H__max'
color_trackbar_Smin = 'S__min'
color_trackbar_Smax = 'S__max'
color_trackbar_Vmin = 'V_min'
color_trackbar_Vmax = 'V_max'

val_Hmin = 74
val_Hmax = 92
val_Smin = 120
val_Smax = 255
val_Vmin = 210
val_Vmax = 255

def imageCallback(ros_data):
    global frame
    frame = bridge.imgmsg_to_cv2(ros_data, "bgr8")

def main():
    global frame
    frame = None

    rospy.Subscriber('my_camera/rgb/image_color', Image, imageCallback,queue_size = 10)
    # pub = rospy.Publisher('')
    while frame is None: pass
    cv2.namedWindow(window_name)
    def nothing(x): pass
    cv2.createTrackbar(color_trackbar_Hmin, window_name, val_Hmin, 180, nothing)
    cv2.createTrackbar(color_trackbar_Hmax, window_name, val_Hmax, 180, nothing)
    cv2.createTrackbar(color_trackbar_Smin, window_name, val_Smin, 255, nothing)
    cv2.createTrackbar(color_trackbar_Smax, window_name, val_Smax, 255, nothing)
    cv2.createTrackbar(color_trackbar_Vmin, window_name, val_Vmin, 255, nothing)
    cv2.createTrackbar(color_trackbar_Vmax, window_name, val_Vmax, 255, nothing)

    while True:
        work_frame = np.copy(frame)
        work_frame = cv2.resize(work_frame, (640, 480))
        work_frame_hsv = cv2.cvtColor(work_frame, cv2.COLOR_BGR2HSV)
        work_frame_hsv = cv2.cvtColor(work_frame, cv2.COLOR_BGR2HSV)
        trackbar_Hmin_pos = cv2.getTrackbarPos(color_trackbar_Hmin, window_name)
        trackbar_Hmax_pos = cv2.getTrackbarPos(color_trackbar_Hmax, window_name)
        trackbar_Smin_pos = cv2.getTrackbarPos(color_trackbar_Smin, window_name)
        trackbar_Smax_pos = cv2.getTrackbarPos(color_trackbar_Smax, window_name)
        trackbar_Vmin_pos = cv2.getTrackbarPos(color_trackbar_Vmin, window_name)
        trackbar_Vmax_pos = cv2.getTrackbarPos(color_trackbar_Vmax, window_name)
        res_frame = cv2.inRange(work_frame_hsv, (trackbar_Hmin_pos, trackbar_Smin_pos, trackbar_Vmin_pos),\
                                                (trackbar_Hmax_pos, trackbar_Smax_pos, trackbar_Vmax_pos))
        # open_kernel = np.ones((3,3), np.uint8)
        close_kernel = np.ones((5, 5), np.uint8)

        # res_frame = cv2.morphologyEx(res_frame, cv2.MORPH_OPEN, open_kernel) # erosion followed by dilation

        res_frame = cv2.morphologyEx(res_frame, cv2.MORPH_CLOSE, close_kernel) # dilation followed by erosion

        # circles = cv2.HoughCircles(res_frame, cv2.HOUGH_GRADIENT, 1, 100)
        # if circles is not None:
        #     print("I'm here")
        #     circles = np.round(circles[0, :]).astype("int")
        #     for (x, y, r) in circles:
        #         cv2.circle(res_frame, (x, y), r, (0, 0, 255), 4)
        #         cv2.rectangle(res_frame, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
        # #
        # ret, thresh = cv2.threshold(res_frame, 127, 255, 0)
        ok_cntr = []
        image, contours, _ = cv2.findContours(res_frame, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        for i in contours:
            area = cv2.contourArea(i)
            if area > 650:
                ok_cntr.append(i)

        cv2.drawContours(work_frame, ok_cntr, -1, (255, 0, 0), 3)
        if len(ok_cntr) > 1:
            print("Achtung! Two green signals!")

        res_bgr_frame = cv2.cvtColor(res_frame, cv2.COLOR_GRAY2BGR)
        res_frame = np.hstack((work_frame, res_bgr_frame))
        cv2.imshow(window_name, res_frame)

        if cv2.waitKey(1) == ord('x'): break
main()
