#!/usr/bin/env python
#  let know what language is used

import rospy
from sensor_msgs.msg import Image
import std_msgs.msg
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import matplotlib.pyplot as plt

window_name = 'roi'
color_trackbar_Hmin = 'H__min'
color_trackbar_Hmax = 'H__max'
color_trackbar_Smin = 'S__min'
color_trackbar_Smax = 'S__max'
color_trackbar_Vmin = 'V_min'
color_trackbar_Vmax = 'V_max'

val_Hmin = 0
val_Hmax = 180
val_Smin = 0
val_Smax = 24
val_Vmin = 191
val_Vmax = 255

rospy.init_node('cross_line')

bridge = CvBridge()

def imageCallback(ros_data):        # get images from rosbag
    global frame
    frame = bridge.imgmsg_to_cv2(ros_data, "bgr8")


# filepath = '/home/user/vlcsnap-2017-07-03-20h45m43s399.png'
def main():
    global frame
    frame = None

    rospy.Subscriber('my_camera/rgb/image_color', Image, imageCallback, queue_size = 10)
    pub_mean = rospy.Publisher('cross_line/mean_perc', std_msgs.msg.Float64, queue_size = 10)
    pub_max = rospy.Publisher('cross_line/max_perc', std_msgs.msg.Float64, queue_size = 10)
    # frame = cv2.imread(filepath)
    while frame is None: pass

    cv2.namedWindow(window_name)
    def nothing(x): pass
    cv2.createTrackbar(color_trackbar_Hmin, window_name, val_Hmin, 180, nothing)
    cv2.createTrackbar(color_trackbar_Hmax, window_name, val_Hmax, 180, nothing)
    cv2.createTrackbar(color_trackbar_Smin, window_name, val_Smin, 255, nothing)
    cv2.createTrackbar(color_trackbar_Smax, window_name, val_Smax, 255, nothing)
    cv2.createTrackbar(color_trackbar_Vmin, window_name, val_Vmin, 255, nothing)
    cv2.createTrackbar(color_trackbar_Vmax, window_name, val_Vmax, 255, nothing)
    # work_frame = cv2.resize(frame, (320, 240))


    while True:
        work_frame = np.copy(frame)
        work_frame = cv2.resize(work_frame, (320, 240))
        work_frame_hsv = cv2.cvtColor(work_frame, cv2.COLOR_BGR2HSV)
        trackbar_Hmin_pos = cv2.getTrackbarPos(color_trackbar_Hmin, window_name)
        trackbar_Hmax_pos = cv2.getTrackbarPos(color_trackbar_Hmax, window_name)
        trackbar_Smin_pos = cv2.getTrackbarPos(color_trackbar_Smin, window_name)
        trackbar_Smax_pos = cv2.getTrackbarPos(color_trackbar_Smax, window_name)
        trackbar_Vmin_pos = cv2.getTrackbarPos(color_trackbar_Vmin, window_name)
        trackbar_Vmax_pos = cv2.getTrackbarPos(color_trackbar_Vmax, window_name)
        res_frame = cv2.inRange(work_frame_hsv, (trackbar_Hmin_pos, trackbar_Smin_pos, trackbar_Vmin_pos),\
                                                (trackbar_Hmax_pos, trackbar_Smax_pos, trackbar_Vmax_pos))
        res_bgr_frame = cv2.cvtColor(res_frame, cv2.COLOR_GRAY2BGR)
        y = range(240)
        white_sum = np.sum(res_frame, axis = 1)
        # print(np.max(white_sum))

        # norm_white_sum = white_sum / float(np.max(white_sum))
        # print(norm_white_sum > 0.8)
        # plt.bar(y, norm_white_sum)
        # plt.plot(1)

        # high_white_value = norm_white_sum > 0.8
        # high_white_value_res = norm_white_sum[high_white_value]
        control_cross = -1
        max_y = -1
        y_arr = []
        for y, val in enumerate(white_sum):
            if val > 30000:
                work_frame = cv2.line(work_frame, (0, y), (360, y), (255, 0, 0), 1)
                y_arr.append(y)
            pass
        if len(y_arr) > 10:
            centr_cross = int(np.mean(y_arr))
            cv2.line(work_frame, (0, centr_cross), (360, centr_cross), (0, 0, 255), 5)
            control_cross = (float(centr_cross) / 240 * 100)
            max_y = float(max(y_arr)) / 240 * 100

        pub_mean.publish(control_cross)
        pub_max.publish(max_y)


        res_frame = np.hstack((work_frame,res_bgr_frame))
        cv2.imshow(window_name, res_frame)

        if cv2.waitKey(1) == ord('x'): break
main()
# plt.show()