#!/usr/bin/env python

import rospy
import std_msgs.msg
import sensor_msgs.msg
import cv2
from cv_bridge import CvBridge, CvBridgeError
from lane_track_config import *
import numpy as np
from lane_search import *

bridge = CvBridge()
frame = None

def image_callback(ros_data):
	global frame
	try:
		frame = bridge.imgmsg_to_cv2(ros_data, "bgr8")
	except CvBridgeError as e:
		rospy.logerror(e)

rospy.init_node('lane_follower')

#-------------------------------------------------

use_topic = rospy.get_param('~use_topic', False)
file_as_video = rospy.get_param('~file_as_video', False)
filepath = rospy.get_param('~filepath', '')

rospy.loginfo('[%s] Param <use_topic>: %s' % (os.path.basename(__file__), str(use_topic)))
rospy.loginfo('[%s] Param <file_as_video>: %s' % (os.path.basename(__file__), str(file_as_video)))
rospy.loginfo('[%s] Param <filepath>: %s' % (os.path.basename(__file__), str(filepath)))

# image_pub = rospy.Publisher('md_control/image', sensor_msgs.msg.Image, queue_size=10)
control_pub = rospy.Publisher('md_control/control', std_msgs.msg.Int32, queue_size=10)

#-------------------------------------------------

window_name = 'result'

visible_frame_size = (640, 480)

def main():
	global frame

	if use_topic:
		rospy.Subscriber('image', sensor_msgs.msg.Image, image_callback,  queue_size = 10)	

	elif file_as_video:
		cap = cv2.VideoCapture(filepath)
		if not cap.isOpened():
			rospy.logerr('Unable to open video file')
			exit(1)

	else:
		frame = cv2.imread(args.filepath)
		if frame is None:
			rospy.logerr('Unable to open image file')
			exit(1)

	cv2.namedWindow(window_name)

	config = LaneTrackConfig()
	roi_desc = config.get_roi_descriptor()
	cf_desc = config.get_color_filter_descriptor()
	lane = LaneSearch()

	while not rospy.is_shutdown():
		if not use_topic:
			if file_as_video:
				ret, frame = cap.read()
		
		if frame is not None:
			work_frame = np.copy(frame)

			work_frame = cv2.resize(work_frame, visible_frame_size)

			work_frame = cv2.GaussianBlur(work_frame,(3,3),0)
			work_frame = cv2.morphologyEx(work_frame, cv2.MORPH_OPEN, np.ones((5,5),np.uint8))

			orig_height, orig_width = get_frame_height_width(work_frame)
			roi_frame = roi_desc.mask_frame_resize(work_frame)
			bin_frame, _ = cf_desc.apply_filters(roi_frame, 50)

			# lane.update_lane_control(bin_frame)

			left_fitx, _, fity = lane.get_lane_polynomials(bin_frame)
			for i in range(len(fity)):
				cv2.circle(roi_frame, (int(left_fitx[i]), int(fity[i])), 1, (0, 255, 0))

			control_right = left_fitx[100]

			# control_right = lane.control_x
			cv2.line(roi_frame, (int(control_right), 0), (int(control_right), orig_height), (255, 255, 0), thickness=3)

			out_frame = cv2.resize(roi_frame, (320, 240))
			# result_frame = np.hstack( (work_frame, roi_frame) );
			cv2.putText(out_frame, str(control_right), (10,70), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,0,0), 3)
			cv2.imshow(window_name, out_frame)
			# cv2.imshow('1', bin_frame)

			cv2.waitKey(1)

			control_pub.publish(control_right)
			# print(control_right)
		else:
			rospy.loginfo('Video closed or error')
			break

	cv2.destroyAllWindows()

if __name__ == '__main__':
	main()
